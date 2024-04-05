/*
 * udp.c
 *
 * Copyright (c) 2023 Jan Rusnak <jan@rusnak.sk>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <gentyp.h>
#include "sysconf.h"
#include "board.h"
#include <mmio.h>
#include "criterr.h"
#include "msgconf.h"
#include "hwerr.h"
#include "pmc.h"
#include "udp.h"

#define UDP_CSR_ACC_TMO 200
#define UDP_CSR_ACC_NOP 5

enum endp_state {
	EP_DISABLED_STATE,
        EP_HALTED_STATE,
	EP_IDLE_STATE,
	EP_ACTIVE_STATE,
	EP_SUSP_STATE
};

enum udp_isr_sig {
	UDP_ISR_SIG_IRP_DONE,
        UDP_ISR_SIG_IRP_BUF_FULL,
	UDP_ISR_SIG_IRP_INTR
};

enum udp_irp_state {
	UDP_IRP_DONE,
	UDP_IRP_PENDING
};

struct endp {
	enum endp_state state;
	short pkt_size;
	int8_t last_bank;
	int8_t banks_nmb;
	boolean_t pp_mode;
        boolean_t use_zero_pkt;
        boolean_t nxt_data;
	uint8_t *buf;
	short cnt;
	short buf_size;
	short min_rcv_nmb;
        QueueHandle_t que;
	enum endp_state wake_state;
	boolean_t wake_intr;
};

struct csr_err {
	int ep;
	unsigned int csr;
	unsigned int sb;
	enum endp_state e_st;
	enum udp_state u_st;
	unsigned int isr;
	int r;
};

#define endp_pkt_size(i)\
	((i == 0) ? 64 :\
	((i == 1) ? 64 :\
	((i == 2) ? 64 :\
	((i == 3) ? 64 :\
	((i == 4) ? 512 :\
	((i == 5) ? 512 :\
	((i == 6) ? 64 :\
	((i == 7) ? 64 : 0))))))))

#define endp_banks_nmb(i)\
	((i == 0) ? 1 :\
	((i == 1) ? 2 :\
	((i == 2) ? 2 :\
	((i == 3) ? 1 :\
	((i == 4) ? 2 :\
	((i == 5) ? 2 :\
	((i == 6) ? 2 :\
	((i == 7) ? 2 : 0))))))))

#define REG_CSR_NO_EFFECT_1_ALL (UDP_CSR_RX_DATA_BK1 | UDP_CSR_STALLSENT |\
		       	         UDP_CSR_RXSETUP | UDP_CSR_RX_DATA_BK0 |\
                                 UDP_CSR_TXCOMP)

#define set_csr(e, f)\
do {\
        volatile unsigned int r69;\
        int tmo69 = UDP_CSR_ACC_TMO;\
	\
        r69 = UDP->UDP_CSR[e];\
        r69 |= REG_CSR_NO_EFFECT_1_ALL;\
        r69 |= (f);\
        UDP->UDP_CSR[e] = r69;\
	for (int cnt69 = 0; cnt69 < UDP_CSR_ACC_NOP; cnt69++) {\
		__asm__ __volatile__ ( "nop" );\
	}\
        while ((UDP->UDP_CSR[e] & (f)) != (f)) {\
		if (!--tmo69) {\
			break;\
		}\
	}\
	if (!tmo69) {\
		stats.set_csr_tmo_cnt++;\
		set_csr_err.ep = e;\
		set_csr_err.csr = UDP->UDP_CSR[e];\
		set_csr_err.sb = f;\
		set_csr_err.e_st = endp_arry[e].state;\
		set_csr_err.u_st = udp_state;\
		set_csr_err.isr = UDP->UDP_ISR;\
		set_csr_err.r = __LINE__;\
	}\
} while (0)

#define clr_csr(e, f)\
do {\
        volatile unsigned int r69;\
        int tmo69 = UDP_CSR_ACC_TMO;\
	\
        r69 = UDP->UDP_CSR[e];\
        r69 |= REG_CSR_NO_EFFECT_1_ALL;\
        r69 &= ~(f);\
        UDP->UDP_CSR[e] = r69;\
	for (int cnt69 = 0; cnt69 < UDP_CSR_ACC_NOP; cnt69++) {\
		__asm__ __volatile__ ( "nop" );\
	}\
        while ((~UDP->UDP_CSR[e] & (f)) != (f)) {\
		if (!--tmo69) {\
			break;\
		}\
	}\
	if (!tmo69) {\
		stats.clr_csr_tmo_cnt++;\
		clr_csr_err.ep = e;\
		clr_csr_err.csr = UDP->UDP_CSR[e];\
		clr_csr_err.sb = f;\
		clr_csr_err.e_st = endp_arry[e].state;\
		clr_csr_err.u_st = udp_state;\
		clr_csr_err.isr = UDP->UDP_ISR;\
		clr_csr_err.r = __LINE__;\
	}\
} while (0)

#define udp_icr_all()\
do {\
	UDP->UDP_ICR = UDP_ICR_WAKEUP | UDP_ICR_ENDBUSRES | UDP_ICR_SOFINT |\
	               UDP_ICR_EXTRSM | UDP_ICR_RXRSM | UDP_ICR_RXSUSP;\
} while (0)

#define udp_idr_all()\
do {\
	UDP->UDP_IDR = UDP_IDR_WAKEUP | UDP_IDR_SOFINT | UDP_IDR_EXTRSM |\
		       UDP_IDR_RXRSM | UDP_IDR_RXSUSP | 0xFF;\
} while (0)

static volatile struct endp endp_arry[UDP_EP_NMB];
static struct udp_stats stats;
static enum udp_state udp_state, udp_last_state;
static void (*sof_intr_clbk)(uint32_t);
static struct csr_err set_csr_err, clr_csr_err;

#if UDP_LOG_INTR_EVENTS == 1 || UDP_LOG_STATE_EVENTS == 1 || UDP_LOG_ENDP_EVENTS == 1 ||\
    UDP_LOG_OUT_IRP_EVENTS == 1 || UDP_LOG_ERR_EVENTS == 1
static logger_t usb_logger;
static BaseType_t dmy;
#endif
#if UDP_LOG_INTR_EVENTS == 1
static const char *susp_str = "susp";
static const char *rsm_str = "rsm";
//static const char *ext_rsm_str = "ersm";
static const char *bus_rst_str = "rst";
static const char *wake_str = "wake";
#endif
#if UDP_LOG_INTR_EVENTS == 1 || UDP_LOG_ERR_EVENTS == 1
static const char *ep_str_arry[] = {"e0", "e1", "e2", "e3", "e4", "e5", "e6", "e7"};
static const char *ep_stp_str = "<stp";
static const char *ep_bk0_str = "<bk0";
static const char *ep_bk1_str = "<bk1";
static const char *ep_bkx_str = "<bkx";
static const char *ep_txc_str = "txc>";
static const char *ep_stls_str = "stls>";
#endif
#if UDP_LOG_ERR_EVENTS == 1
static const char *ep_dise_str = "dise";
static const char *unexp_rst_str = "unexp_rst";
static const char *bad_csr_str = "bad_csr";
static const char *que_err_str = "que_err";
static const char *unexp_intr_str = "unexp_intr";
static const char *nfstal_str = "nfstal_str";
static const char *rx_on_inact_ep_str = "rx_on_inact_ep";
static const char *rx_on_in_ep_str = "rx_on_in_ep";
static const char *txc_on_inact_ep_str = "txc_on_inact_ep";
static const char *txc_on_out_ep_str = "txc_on_out_ep";
static const char *pkt_sz_str = "pkt_sz";
#endif
#if UDP_LOG_ENDP_EVENTS == 1
static const char *ep_ev_ena_str = "enable";
static const char *ep_ev_dis_str = "disable";
static const char *ep_ev_hlt_str = "halt";
static const char *ep_ev_unhlt_str = "unhalt";
#endif
#if UDP_LOG_OUT_IRP_EVENTS == 1
static const char *oirp_sh_pkt_str = "shp";
static const char *oirp_bf_ful_str = "bff";
static const char *oirp_mx_dt_str = "mxd";
#endif
static void (*endp0_rxstp_clbk)(void);
static void (*endp0_txcomp_clbk)(void);
static void (*endp0_stlsnt_clbk)(void);
static void (*endp0_rxdata_clbk)(int);
static boolean_t rmt_wkup_feat;
static BaseType_t task_wkn;
static QueueHandle_t evnt_que;
#if configUSE_QUEUE_SETS == 1
static QueueSetHandle_t evnt_que_qset;
#endif

static void clear_rx_data(int epn);
static void clear_tx_data(int epn);
static void endp0_isr(void);
static void endpn_isr(int epn);
static void txcomp(int epn);
static void txcomp_ppm(int epn);
static void txcomp_no_ppm(int epn);
static void rxdata(int epn);
static enum udp_irp_state rxdata_bnk(int epn, unsigned int bnk);
static void irp_done(int epn, enum udp_isr_sig sig);
static void notif_state_event(void);
#if TERMOUT == 1
static const char *eps(int ep);
#endif
#if UDP_LOG_INTR_EVENTS == 1
static void log_intr_event(const char *intr, const char *csr);
#endif
#if UDP_LOG_STATE_EVENTS == 1
static void log_state_event(const char *txt);
#endif
#if UDP_LOG_ENDP_EVENTS == 1
static void log_endp_event(int epn, const char *txt);
#endif
#if UDP_LOG_OUT_IRP_EVENTS == 1
static void log_out_irp_event(int epn, unsigned int bnk, int sz, int cnt, const char *txt);
#endif
#if UDP_LOG_ERR_EVENTS == 1
static void log_err_event(const char *intr, const char *err, const char *txt);
static void log_err_event_nisr(const char *intr, const char *err, const char *txt);
#endif

/**
 * init_udp
 */
void init_udp(logger_t *logger)
{
#if UDP_LOG_INTR_EVENTS == 1 || UDP_LOG_STATE_EVENTS == 1 || UDP_LOG_ENDP_EVENTS == 1 ||\
    UDP_LOG_OUT_IRP_EVENTS == 1 || UDP_LOG_ERR_EVENTS == 1
	usb_logger = *logger;
#endif
	evnt_que = xQueueCreate(UDP_EVNT_QUE_SIZE, sizeof(enum udp_state));
	if (evnt_que == NULL) {
		crit_err_exit(MALLOC_ERROR);
	}
#if configUSE_QUEUE_SETS == 1
	if (evnt_que_qset) {
		if (pdFAIL == xQueueAddToSet(evnt_que, evnt_que_qset)) {
			crit_err_exit(UNEXP_PROG_STATE);
		}
	}
#endif
        udp_state = UDP_STATE_POWERED;
#if UDP_LOG_STATE_EVENTS == 1
	log_state_event("");
#endif
	if (!(endp0_rxstp_clbk && endp0_txcomp_clbk && endp0_stlsnt_clbk && endp0_rxdata_clbk)) {
		crit_err_exit(BAD_PARAMETER);
	}
	enable_periph_clk(ID_UDP);
        enable_udp_48mhz_clk(USB_UDP_PLL_UNIT, USB_UDP_PLL_MUL, USB_UDP_PLL_DIV, USB_UDP_PLL_DIV2,
			     CKGR_PLL_LOCK_COUNT, USB_UDP_USBCLK_DIV);
        UDP->UDP_TXVC &= ~UDP_TXVC_PUON;
        UDP->UDP_TXVC |= UDP_TXVC_TXVDIS;
	udp_idr_all();
        udp_icr_all();
	NVIC_ClearPendingIRQ(ID_UDP);
	NVIC_SetPriority(ID_UDP, configLIBRARY_MAX_API_CALL_INTERRUPT_PRIORITY);
	NVIC_EnableIRQ(ID_UDP);
}

/**
 * get_udp_evnt_que
 */
QueueHandle_t get_udp_evnt_que(void)
{
	if (!evnt_que) {
		crit_err_exit(UNEXP_PROG_STATE);
	}
	return (evnt_que);
}

#if configUSE_QUEUE_SETS == 1
/**
 * add_udp_evnt_que_to_qset
 */
void add_udp_evnt_que_to_qset(QueueSetHandle_t qset)
{
	evnt_que_qset = qset;
}
#endif

/**
 * udp_in_irp
 */
int udp_in_irp(int epn, void *buf, int size, boolean_t use_zero_pkt)
{
	enum udp_isr_sig sig;
        volatile struct endp *ep;
	int sz;

	if (!size) {
		return (0);
	}
	ep = &endp_arry[epn];
	taskENTER_CRITICAL();
	if (ep->state != EP_IDLE_STATE) {
		taskEXIT_CRITICAL();
		return (-ENRDY);
	}
	if (!(UDP->UDP_CSR[epn] & UDP_CSR_EPEDS)) {
		taskEXIT_CRITICAL();
#if UDP_LOG_ERR_EVENTS == 1
		log_err_event_nisr(ep_str_arry[epn], unexp_rst_str, "");
#endif
		stats.unexp_ep_rst_cnt++;
		return (-ENRDY);
	}
	if (use_zero_pkt) {
		ep->use_zero_pkt = (size % ep->pkt_size) ? FALSE : TRUE;
	} else {
		ep->use_zero_pkt = FALSE;
	}
	if (size >= ep->pkt_size) {
		sz = ep->pkt_size;
	} else {
		sz = size;
	}
	for (int i = 0; i < sz; i++) {
		UDP->UDP_FDR[epn] = *((uint8_t *) buf + i);
	}
        set_csr(epn, UDP_CSR_TXPKTRDY);
	ep->buf = (uint8_t *) buf + sz;
	ep->cnt = size - sz;
	if (ep->pp_mode && ep->cnt > 0) {
		if (ep->cnt >= ep->pkt_size) {
			sz = ep->pkt_size;
		} else {
			sz = ep->cnt;
		}
                for (int i = 0; i < sz; i++) {
			UDP->UDP_FDR[epn] = *((uint8_t *) ep->buf + i);
		}
		ep->buf += sz;
		ep->cnt -= sz;
		ep->nxt_data = TRUE;
	} else {
		ep->nxt_data = FALSE;
	}
	ep->state = EP_ACTIVE_STATE;
        UDP->UDP_IER = 1 << epn;
        taskEXIT_CRITICAL();
	xQueueReceive(ep->que, &sig, portMAX_DELAY);
	switch (sig) {
	case UDP_ISR_SIG_IRP_DONE :
		return (0);
	default :
		return (-EINTR);
	}
}

/**
 * udp_out_irp
 */
int udp_out_irp(int epn, void *buf, int size, int min_rcv_nmb, int *rcv_nmb)
{
	enum udp_isr_sig sig;
        volatile struct endp *ep;

	ep = &endp_arry[epn];
	taskENTER_CRITICAL();
	if (ep->state != EP_IDLE_STATE) {
		taskEXIT_CRITICAL();
		return (-ENRDY);
	}
	if (!(UDP->UDP_CSR[epn] & UDP_CSR_EPEDS)) {
		taskEXIT_CRITICAL();
#if UDP_LOG_ERR_EVENTS == 1
		log_err_event_nisr(ep_str_arry[epn], unexp_rst_str, "");
#endif
		stats.unexp_ep_rst_cnt++;
		return (-ENRDY);
	}
	ep->buf = buf;
	ep->buf_size = size;
	ep->min_rcv_nmb = min_rcv_nmb;
	ep->cnt = 0;
	ep->state = EP_ACTIVE_STATE;
        UDP->UDP_IER = 1 << epn;
        taskEXIT_CRITICAL();
	xQueueReceive(ep->que, &sig, portMAX_DELAY);
	*rcv_nmb = ep->cnt;
	switch (sig) {
	case UDP_ISR_SIG_IRP_DONE     :
		return (0);
	case UDP_ISR_SIG_IRP_BUF_FULL :
		return (-EBFOV);
	default :
		return (-EINTR);
	}
}

/**
 * init_udp_endp_que
 */
void init_udp_endp_que(int epn)
{
	if (epn > 0 && epn < UDP_EP_NMB) {
		endp_arry[epn].que = xQueueCreate(1, sizeof(enum udp_isr_sig));
		if (endp_arry[epn].que == NULL) {
			crit_err_exit(MALLOC_ERROR);
		}
	} else {
		crit_err_exit(BAD_PARAMETER);
	}
}

/**
 * enable_udp_endp
 */
void enable_udp_endp(int epn, enum udp_endp_type etp)
{
	if (endp_arry[epn].state == EP_DISABLED_STATE) {
		endp_arry[epn].pkt_size = endp_pkt_size(epn);
		endp_arry[epn].last_bank = 1;
		endp_arry[epn].banks_nmb = endp_banks_nmb(epn);
		switch (etp) {
		case UDP_ISO_OUT_ENDP  :
			/* FALLTHRU */
		case UDP_BULK_OUT_ENDP :
			/* FALLTHRU */
		case UDP_ISO_IN_ENDP   :
			/* FALLTHRU */
		case UDP_BULK_IN_ENDP  :
			if (endp_arry[epn].banks_nmb == 2) {
				endp_arry[epn].pp_mode = TRUE;
			} else {
				endp_arry[epn].pp_mode = FALSE;
			}
			break;
		default	:
			endp_arry[epn].pp_mode = FALSE;
			break;
		}
		clr_csr(epn, UDP_CSR_EPTYPE_Msk);
		set_csr(epn, UDP_CSR_EPEDS | (etp << UDP_CSR_EPTYPE_Pos));
		clr_csr(epn, UDP_CSR_STALLSENT | UDP_CSR_TXCOMP);
		while (UDP->UDP_CSR[epn] & (UDP_CSR_RX_DATA_BK0 | UDP_CSR_RX_DATA_BK1)) {
			clr_csr(epn, UDP_CSR_RX_DATA_BK0 | UDP_CSR_RX_DATA_BK1);
		}
		UDP->UDP_RST_EP |= 1 << epn;
		while (!(UDP->UDP_RST_EP & (1 << epn)));
		UDP->UDP_RST_EP &= ~(1 << epn);
		if (epn == 0) {
			UDP->UDP_IER = 1 << 0;
		}
		endp_arry[epn].state = EP_IDLE_STATE;
#if UDP_LOG_ENDP_EVENTS == 1
		log_endp_event(epn, ep_ev_ena_str);
#endif
	} else {
		stats.unexp_endp_enab_cnt++;
	}
}

/**
 * disable_udp_endp
 */
void disable_udp_endp(int epn)
{
	enum udp_isr_sig sig;
	BaseType_t wkn;

	UDP->UDP_IDR = 1 << epn;
	if (endp_arry[epn].state == EP_ACTIVE_STATE) {
#if UDP_DBG_ISR_QUE == 1
		if (pdTRUE == xQueueReceiveFromISR(endp_arry[epn].que, &sig, NULL)) {
#if UDP_LOG_ERR_EVENTS == 1
			log_err_event(ep_str_arry[epn], que_err_str, "");
#endif
			stats.isr_que_err_cnt++;
		}
#endif
		sig = UDP_ISR_SIG_IRP_INTR;
		wkn = pdFALSE;
		xQueueSendFromISR(endp_arry[epn].que, &sig, &wkn);
		if (wkn == pdTRUE) {
			task_wkn = pdTRUE;
		}
	}
	clr_csr(epn, UDP_CSR_RXSETUP);
	clear_rx_data(epn);
	clear_tx_data(epn);
	clr_csr(epn, UDP_CSR_FORCESTALL);
	clr_csr(epn, UDP_CSR_STALLSENT);
	clr_csr(epn, UDP_CSR_EPEDS);
	clr_csr(epn, UDP_CSR_EPTYPE_Msk);
	endp_arry[epn].state = EP_DISABLED_STATE;
	if (UDP->UDP_CSR[epn] & (UDP_CSR_DTGLE | UDP_CSR_RX_DATA_BK1 | UDP_CSR_FORCESTALL |
	    UDP_CSR_TXPKTRDY | UDP_CSR_STALLSENT | UDP_CSR_RXSETUP | UDP_CSR_RX_DATA_BK0 | UDP_CSR_TXCOMP)) {
#if UDP_LOG_ERR_EVENTS == 1
		log_err_event(ep_str_arry[epn], ep_dise_str, bad_csr_str);
#endif
		stats.ep_dise_cnt++;
	}
#if UDP_LOG_ENDP_EVENTS == 1
	log_endp_event(epn, ep_ev_dis_str);
#endif
}

/**
 * halt_udp_endp
 */
void halt_udp_endp(int epn)
{
	enum udp_isr_sig sig;
	BaseType_t wkn;

	if (epn != 0 && endp_arry[epn].state != EP_DISABLED_STATE &&
	    endp_arry[epn].state != EP_HALTED_STATE) {
		UDP->UDP_IER = 1 << epn;
		if (endp_arry[epn].state == EP_ACTIVE_STATE) {
#if UDP_DBG_ISR_QUE == 1
			if (pdTRUE == xQueueReceiveFromISR(endp_arry[epn].que, &sig, NULL)) {
#if UDP_LOG_ERR_EVENTS == 1
				log_err_event(ep_str_arry[epn], que_err_str, "");
#endif
				stats.isr_que_err_cnt++;
			}
#endif
			sig = UDP_ISR_SIG_IRP_INTR;
                        wkn = pdFALSE;
			xQueueSendFromISR(endp_arry[epn].que, &sig, &wkn);
			if (wkn == pdTRUE) {
				task_wkn = pdTRUE;
			}
		}
		set_csr(epn, UDP_CSR_FORCESTALL);
		endp_arry[epn].state = EP_HALTED_STATE;
#if UDP_LOG_ENDP_EVENTS == 1
		log_endp_event(epn, ep_ev_hlt_str);
#endif
	} else {
		stats.unexp_endp_hlt_cnt++;
	}
}

/**
 * un_halt_udp_endp
 */
void un_halt_udp_endp(int epn)
{
	if (epn != 0 && endp_arry[epn].state == EP_HALTED_STATE) {
		if (UDP_ENDP_DIR_OUT == get_udp_endp_dir(epn)) {
			clear_rx_data(epn);
			endp_arry[epn].last_bank = 1;
		} else {
			clear_tx_data(epn);
		}
		clr_csr(epn, UDP_CSR_FORCESTALL);
		clr_csr(epn, UDP_CSR_STALLSENT);
		endp_arry[epn].state = EP_IDLE_STATE;
#if UDP_LOG_ENDP_EVENTS == 1
		log_endp_event(epn, ep_ev_unhlt_str);
#endif
	} else {
		stats.unexp_endp_unhlt_cnt++;
	}
}

/**
 * clear_rx_data
 */
static void clear_rx_data(int epn)
{
	while (UDP->UDP_CSR[epn] & (UDP_CSR_RX_DATA_BK0 | UDP_CSR_RX_DATA_BK1)) {
		clr_csr(epn, UDP_CSR_RX_DATA_BK0 | UDP_CSR_RX_DATA_BK1);
	}
	UDP->UDP_RST_EP |= 1 << epn;
	while (!(UDP->UDP_RST_EP & (1 << epn)));
	UDP->UDP_RST_EP &= ~(1 << epn);
}

/**
 * clear_tx_data
 */
static void clear_tx_data(int epn)
{
	clr_csr(epn, UDP_CSR_TXCOMP);
	set_csr(epn, UDP_CSR_TXPKTRDY);
	clr_csr(epn, UDP_CSR_TXPKTRDY);
	set_csr(epn, UDP_CSR_TXPKTRDY);
	clr_csr(epn, UDP_CSR_TXPKTRDY);
	clr_csr(epn, UDP_CSR_TXCOMP);
	UDP->UDP_RST_EP |= 1 << epn;
	while (!(UDP->UDP_RST_EP & (1 << epn)));
	UDP->UDP_RST_EP &= ~(1 << epn);
}

/**
 * UDP_Handler
 */
void UDP_Handler(void)
{
	unsigned int isr;

        task_wkn = pdFALSE;
	stats.intr_cnt++;
	isr = UDP->UDP_ISR;
	isr &= UDP->UDP_IMR;
	if (isr & UDP_ISR_SOFINT) {
		UDP->UDP_ICR = UDP_ICR_SOFINT;
		if (sof_intr_clbk) {
			sof_intr_clbk(UDP->UDP_FRM_NUM);
                        goto isr_exit;
		}
	}
	if (isr & UDP_ISR_ENDBUSRES) {
#if UDP_LOG_INTR_EVENTS == 1
		log_intr_event(bus_rst_str, NULL);
#endif
		stats.brst_cnt++;
		if (udp_state == UDP_STATE_SUSPENDED) {
                        enable_periph_clk_nocs(ID_UDP);
			enable_udp_48mhz_clk(USB_UDP_PLL_UNIT, USB_UDP_PLL_MUL,
			                     USB_UDP_PLL_DIV, USB_UDP_PLL_DIV2,
					     CKGR_PLL_LOCK_COUNT, USB_UDP_USBCLK_DIV);
			for (int i = 1; i < UDP_EP_NMB; i++) {
				if (endp_arry[i].state != EP_DISABLED_STATE) {
					endp_arry[i].state = endp_arry[i].wake_state;
				}
			}
		}
                udp_state = UDP_STATE_DEFAULT;
#if UDP_LOG_STATE_EVENTS == 1
		log_state_event("");
#endif
		notif_state_event();
                UDP->UDP_TXVC &= ~UDP_TXVC_TXVDIS;
                udp_icr_all();
		udp_idr_all();
                UDP->UDP_IER = UDP_IER_RXSUSP;
		if (sof_intr_clbk) {
			UDP->UDP_IER = UDP_IER_SOFINT;
		}
		for (int i = 0; i < UDP_EP_NMB; i++) {
			disable_udp_endp(i);
		}
                UDP->UDP_GLB_STAT &= ~(UDP_GLB_STAT_RMWUPE | UDP_GLB_STAT_CONFG);
		UDP->UDP_FADDR = UDP_FADDR_FEN;
                rmt_wkup_feat = FALSE;
		enable_udp_endp(0, UDP_CTRL_ENDP);
	} else if (isr == UDP_ISR_RXSUSP) {
#if UDP_LOG_INTR_EVENTS == 1
		log_intr_event(susp_str, NULL);
#endif
		stats.susp_cnt++;
		udp_last_state = udp_state;
		udp_state = UDP_STATE_SUSPENDED;
#if UDP_LOG_STATE_EVENTS == 1
		log_state_event("");
#endif
		notif_state_event();
		UDP->UDP_IER = UDP_IER_WAKEUP | UDP_IER_RXRSM;
		UDP->UDP_IDR = UDP_IDR_RXSUSP;
		UDP->UDP_ICR = UDP_ICR_RXSUSP;
		UDP->UDP_IDR = 1 << 0;
		for (int i = 1; i < UDP_EP_NMB; i++) {
			if (endp_arry[i].state != EP_DISABLED_STATE) {
				endp_arry[i].wake_state = endp_arry[i].state;
				endp_arry[i].state = EP_SUSP_STATE;
				if (UDP->UDP_IMR & (1 << i)) {
					endp_arry[i].wake_intr = TRUE;
					UDP->UDP_IDR = 1 << i;
				} else {
					endp_arry[i].wake_intr = FALSE;
				}
			}
		}
		UDP->UDP_TXVC |= UDP_TXVC_TXVDIS;
		disable_udp_48mhz_clk(USB_UDP_PLL_UNIT);
		disable_periph_clk_nocs(ID_UDP);
	} else if (isr & (UDP_ISR_WAKEUP | UDP_ISR_RXRSM)) {
#if UDP_LOG_INTR_EVENTS == 1
		log_intr_event((isr & UDP_ISR_WAKEUP) ? wake_str : rsm_str, NULL);
#endif
		stats.wake_cnt++;
		if (udp_state == UDP_STATE_SUSPENDED) {
			udp_state = udp_last_state;
#if UDP_LOG_STATE_EVENTS == 1
			log_state_event("waked");
#endif
			notif_state_event();
                        enable_periph_clk_nocs(ID_UDP);
			enable_udp_48mhz_clk(USB_UDP_PLL_UNIT, USB_UDP_PLL_MUL,
			                     USB_UDP_PLL_DIV, USB_UDP_PLL_DIV2,
					     CKGR_PLL_LOCK_COUNT, USB_UDP_USBCLK_DIV);
			if (udp_state >= UDP_STATE_DEFAULT) {
				UDP->UDP_TXVC &= ~UDP_TXVC_TXVDIS;
			}
			UDP->UDP_IER = 1 << 0;
			for (int i = 1; i < UDP_EP_NMB; i++) {
				if (endp_arry[i].state != EP_DISABLED_STATE) {
					endp_arry[i].state = endp_arry[i].wake_state;
					if (endp_arry[i].wake_intr) {
						UDP->UDP_IER = 1 << i;
					}
				}
			}
		} else {
			stats.unexp_wake_cnt++;
		}
                UDP->UDP_IER = UDP_IER_RXSUSP;
                UDP->UDP_IDR = UDP_IDR_WAKEUP | UDP_IDR_RXRSM;
		UDP->UDP_ICR = UDP_ICR_WAKEUP | UDP_ICR_RXRSM | UDP_ICR_RXSUSP;
	} else {
		if (isr & (1 << 0)) {
			endp0_isr();
		} else {
			for (int i = 1; i < UDP_EP_NMB; i++) {
				if (isr & (1 << i)) {
					endpn_isr(i);
				}
			}
		}
	}
isr_exit:
        portEND_SWITCHING_ISR(task_wkn);
}

/**
 * endp0_isr
 */
static void endp0_isr(void)
{
	if (UDP->UDP_CSR[0] & UDP_CSR_RXSETUP) {
#if UDP_LOG_INTR_EVENTS == 1
		log_intr_event(ep_str_arry[0], ep_stp_str);
#endif
		endp0_rxstp_clbk();
	} else if (UDP->UDP_CSR[0] & UDP_CSR_TXCOMP) {
#if UDP_LOG_INTR_EVENTS == 1
		log_intr_event(ep_str_arry[0], ep_txc_str);
#endif
                endp0_txcomp_clbk();
	} else if (UDP->UDP_CSR[0] & UDP_CSR_STALLSENT) {
#if UDP_LOG_INTR_EVENTS == 1
		log_intr_event(ep_str_arry[0], ep_stls_str);
#endif
		stats.stls_cnt[0]++;
		endp0_stlsnt_clbk();
	} else {
		if (UDP->UDP_CSR[0] & UDP_CSR_RX_DATA_BK0) {
#if UDP_LOG_INTR_EVENTS == 1
			log_intr_event(ep_str_arry[0], ep_bk0_str);
#endif
			endp0_rxdata_clbk((UDP->UDP_CSR[0] & UDP_CSR_RXBYTECNT_Msk) >>
			                   UDP_CSR_RXBYTECNT_Pos);
		} else {
#if UDP_LOG_ERR_EVENTS == 1
			log_err_event(ep_str_arry[0], bad_csr_str, "");
#endif
			stats.bad_csr_flags_isr_cnt++;
			clr_csr(0, UDP_CSR_RX_DATA_BK1);
		}
	}
}

/**
 * endpn_isr
 */
static void endpn_isr(int epn)
{
	if (UDP->UDP_CSR[epn] & UDP_CSR_RXSETUP) {
#if UDP_LOG_ERR_EVENTS == 1
		log_err_event(ep_str_arry[epn], unexp_intr_str, ep_stp_str);
#endif
		stats.unexp_rxstp_cnt++;
		clr_csr(epn, UDP_CSR_RXSETUP);
		return;
	}
	if (UDP->UDP_CSR[epn] & UDP_CSR_STALLSENT) {
		if (endp_arry[epn].state == EP_HALTED_STATE) {
#if UDP_LOG_INTR_EVENTS == 1
			log_intr_event(ep_str_arry[epn], ep_stls_str);
#endif
			stats.stls_cnt[epn]++;
			if (!(UDP->UDP_CSR[epn] & UDP_CSR_FORCESTALL)) {
#if UDP_LOG_ERR_EVENTS == 1
				log_err_event(ep_str_arry[epn], nfstal_str, ep_stls_str);
#endif
				stats.nfstal_cnt++;
			}
		} else {
#if UDP_LOG_ERR_EVENTS == 1
		        log_err_event(ep_str_arry[epn], unexp_intr_str, ep_stls_str);
#endif
			stats.unexp_stls_cnt++;
		}
		clr_csr(epn, UDP_CSR_STALLSENT);
		return;
	}
	if (UDP->UDP_CSR[epn] & UDP_CSR_TXCOMP) {
		txcomp(epn);
	} else if (UDP->UDP_CSR[epn] & (UDP_CSR_RX_DATA_BK0 | UDP_CSR_RX_DATA_BK1)) {
		rxdata(epn);
	} else {
#if UDP_LOG_ERR_EVENTS == 1
		log_err_event(ep_str_arry[epn], bad_csr_str, "");
#endif
		stats.bad_csr_flags_isr_cnt++;
	}
}

/**
 * txcomp
 */
static void txcomp(int epn)
{
	if (get_udp_endp_dir(epn) != UDP_ENDP_DIR_IN) {
#if UDP_LOG_ERR_EVENTS == 1
	        log_err_event(ep_str_arry[epn], txc_on_out_ep_str, ep_txc_str);
#endif
		stats.txc_on_out_ep_cnt++;
		clr_csr(epn, UDP_CSR_TXCOMP);
		halt_udp_endp(epn);
		return;
	}
	if (endp_arry[epn].state != EP_ACTIVE_STATE) {
		if (endp_arry[epn].state == EP_HALTED_STATE) {
			clr_csr(epn, UDP_CSR_TXCOMP);
			return;
		}
#if UDP_LOG_ERR_EVENTS == 1
	        log_err_event(ep_str_arry[epn], txc_on_inact_ep_str, ep_txc_str);
#endif
		stats.txc_on_inact_ep_cnt++;
		clr_csr(epn, UDP_CSR_TXCOMP);
		return;
	}
#if UDP_LOG_INTR_EVENTS == 1
	log_intr_event(ep_str_arry[epn], ep_txc_str);
#endif
	if (endp_arry[epn].pp_mode) {
		txcomp_ppm(epn);
	} else {
		txcomp_no_ppm(epn);
	}
}

/**
 * txcomp_ppm
 */
static void txcomp_ppm(int epn)
{
	volatile struct endp *ep;
	int sz;

	ep = &endp_arry[epn];
	if (ep->nxt_data) {
		set_csr(epn, UDP_CSR_TXPKTRDY);
                clr_csr(epn, UDP_CSR_TXCOMP);
                if (ep->cnt > 0) {
			if (ep->cnt >= ep->pkt_size) {
				sz = ep->pkt_size;
			} else {
				sz = ep->cnt;
			}
			for (int i = 0; i < sz; i++) {
				UDP->UDP_FDR[epn] = *((uint8_t *) ep->buf + i);
			}
			ep->buf += sz;
			ep->cnt -= sz;
		} else {
			ep->nxt_data = FALSE;
		}
	} else {
		if (ep->use_zero_pkt) {
			ep->use_zero_pkt = FALSE;
                        set_csr(epn, UDP_CSR_TXPKTRDY);
		} else {
			irp_done(epn, UDP_ISR_SIG_IRP_DONE);
		}
                clr_csr(epn, UDP_CSR_TXCOMP);
	}
}

/**
 * txcomp_no_ppm
 */
static void txcomp_no_ppm(int epn)
{
	volatile struct endp *ep;
	int sz;

	ep = &endp_arry[epn];
	if (ep->cnt == 0) {
		if (ep->use_zero_pkt) {
			ep->use_zero_pkt = FALSE;
                        set_csr(epn, UDP_CSR_TXPKTRDY);
		} else {
			irp_done(epn, UDP_ISR_SIG_IRP_DONE);
		}
	} else {
		if (ep->cnt >= ep->pkt_size) {
			sz = ep->pkt_size;
		} else {
			sz = ep->cnt;
		}
                for (int i = 0; i < sz; i++) {
			UDP->UDP_FDR[epn] = *((uint8_t *) ep->buf + i);
		}
                set_csr(epn, UDP_CSR_TXPKTRDY);
		ep->buf += sz;
		ep->cnt -= sz;
	}
        clr_csr(epn, UDP_CSR_TXCOMP);
}

/**
 * rxdata
 */
static void rxdata(int epn)
{
	unsigned int csr = UDP->UDP_CSR[epn];

	if (get_udp_endp_dir(epn) != UDP_ENDP_DIR_OUT) {
#if UDP_LOG_ERR_EVENTS == 1
	        log_err_event(ep_str_arry[epn], rx_on_in_ep_str, ep_bkx_str);
#endif
		stats.rx_on_in_ep_cnt++;
		clr_csr(epn, UDP_CSR_RX_DATA_BK0 | UDP_CSR_RX_DATA_BK1);
		halt_udp_endp(epn);
		return;
	}
	if (endp_arry[epn].state != EP_ACTIVE_STATE) {
		if (endp_arry[epn].state == EP_HALTED_STATE) {
			clr_csr(epn, UDP_CSR_RX_DATA_BK0 | UDP_CSR_RX_DATA_BK1);
			return;
		}
#if UDP_LOG_ERR_EVENTS == 1
	        log_err_event(ep_str_arry[epn], rx_on_inact_ep_str, ep_bkx_str);
#endif
		stats.rx_on_inact_ep_cnt++;
		clr_csr(epn, UDP_CSR_RX_DATA_BK0 | UDP_CSR_RX_DATA_BK1);
		return;
	}
	if (endp_arry[epn].pp_mode) {
		if (csr & UDP_CSR_RX_DATA_BK0 && csr & UDP_CSR_RX_DATA_BK1) {
#if UDP_LOG_INTR_EVENTS == 1
			log_intr_event(ep_str_arry[epn], ep_bkx_str);
#endif
			if (endp_arry[epn].last_bank == 1) {
				if (UDP_IRP_DONE == rxdata_bnk(epn, UDP_CSR_RX_DATA_BK0)) {
					return;
				}
				(void) rxdata_bnk(epn, UDP_CSR_RX_DATA_BK1);
			} else {
				if (UDP_IRP_DONE == rxdata_bnk(epn, UDP_CSR_RX_DATA_BK1)) {
					return;
				}
				(void) rxdata_bnk(epn, UDP_CSR_RX_DATA_BK0);
			}
		} else if (csr & UDP_CSR_RX_DATA_BK0 && endp_arry[epn].last_bank == 1) {
			(void) rxdata_bnk(epn, UDP_CSR_RX_DATA_BK0);
		} else if (csr & UDP_CSR_RX_DATA_BK1 && endp_arry[epn].last_bank == 0) {
			(void) rxdata_bnk(epn, UDP_CSR_RX_DATA_BK1);
		} else {
#if UDP_LOG_ERR_EVENTS == 1
			log_err_event(ep_str_arry[epn], bad_csr_str, ep_bkx_str);
#endif
			stats.data_bkx_hwerr_cnt++;
			clr_csr(epn, UDP_CSR_RX_DATA_BK0 | UDP_CSR_RX_DATA_BK1);
			halt_udp_endp(epn);
		}
	} else {
		if (csr & UDP_CSR_RX_DATA_BK0) {
#if UDP_LOG_INTR_EVENTS == 1
			log_intr_event(ep_str_arry[epn], ep_bk0_str);
#endif
			(void) rxdata_bnk(epn, UDP_CSR_RX_DATA_BK0);
		} else {
#if UDP_LOG_ERR_EVENTS == 1
			log_err_event(ep_str_arry[epn], bad_csr_str, "");
#endif
			stats.bad_csr_flags_isr_cnt++;
			clr_csr(epn, UDP_CSR_RX_DATA_BK0 | UDP_CSR_RX_DATA_BK1);
			halt_udp_endp(epn);
		}
	}
}

/**
 * rxdata_bnk
 */
static enum udp_irp_state rxdata_bnk(int epn, unsigned int bnk)
{
	volatile struct endp *ep;
	int sz;
        enum udp_irp_state ret;

	ret = UDP_IRP_DONE;
	ep = &endp_arry[epn];
	sz = (UDP->UDP_CSR[epn] & UDP_CSR_RXBYTECNT_Msk) >> UDP_CSR_RXBYTECNT_Pos;
	if (sz > ep->pkt_size) {
#if UDP_LOG_ERR_EVENTS == 1
	        log_err_event(ep_str_arry[epn], pkt_sz_str,
			     (bnk == UDP_CSR_RX_DATA_BK0) ? ep_bk0_str : ep_bk1_str);
#endif
		stats.bad_pkt_sz_cnt++;
		clr_csr(epn, UDP_CSR_RX_DATA_BK0 | UDP_CSR_RX_DATA_BK1);
		halt_udp_endp(epn);
		return (ret);
	}
	if (ep->cnt + sz > ep->buf_size) {
#if UDP_LOG_OUT_IRP_EVENTS == 1
		log_out_irp_event(epn, bnk, sz, ep->cnt, oirp_bf_ful_str);
#endif
		irp_done(epn, UDP_ISR_SIG_IRP_BUF_FULL);
		return (ret);
	}
	for (int i = 0; i < sz; i++) {
		*ep->buf++ = UDP->UDP_FDR[epn];
	}
        ep->cnt += sz;
	if (sz < ep->pkt_size) {
#if UDP_LOG_OUT_IRP_EVENTS == 1
		log_out_irp_event(epn, bnk, sz, ep->cnt, oirp_sh_pkt_str);
#endif
                irp_done(epn, UDP_ISR_SIG_IRP_DONE);
	} else {
		if (ep->min_rcv_nmb != 0 && ep->cnt >= ep->min_rcv_nmb) {
#if UDP_LOG_OUT_IRP_EVENTS == 1
			log_out_irp_event(epn, bnk, sz, ep->cnt, oirp_mx_dt_str);
#endif
			irp_done(epn, UDP_ISR_SIG_IRP_DONE);
		} else {
#if UDP_LOG_OUT_IRP_EVENTS == 1 && UDP_LOG_OUT_IRP_EVENTS_ALL == 1
			log_out_irp_event(epn, bnk, sz, ep->cnt, NULL);
#endif
			ret = UDP_IRP_PENDING;
		}
	}
	if (bnk == UDP_CSR_RX_DATA_BK0) {
		clr_csr(epn, UDP_CSR_RX_DATA_BK0);
                if (ep->pp_mode) {
			ep->last_bank = 0;
		}
	} else {
		clr_csr(epn, UDP_CSR_RX_DATA_BK1);
		ep->last_bank = 1;
	}
	return (ret);
}

/**
 * irp_done
 */
static void irp_done(int epn, enum udp_isr_sig sig)
{
	BaseType_t wkn = pdFALSE;

#if UDP_DBG_ISR_QUE == 1
	enum udp_isr_sig tmp;
	if (pdTRUE == xQueueReceiveFromISR(endp_arry[epn].que, &tmp, NULL)) {
#if UDP_LOG_ERR_EVENTS == 1
		log_err_event(ep_str_arry[epn], que_err_str, "");
#endif
		stats.isr_que_err_cnt++;
	}
#endif
	xQueueSendFromISR(endp_arry[epn].que, &sig, &wkn);
	if (wkn == pdTRUE) {
		task_wkn = pdTRUE;
	}
	UDP->UDP_IDR = 1 << epn;
	endp_arry[epn].state = EP_IDLE_STATE;
}

/**
 * notif_state_event
 */
static void notif_state_event(void)
{
	BaseType_t wkn = pdFALSE;

	if (pdTRUE == xQueueSendFromISR(evnt_que, &udp_state, &wkn)) {
		if (wkn == pdTRUE) {
			task_wkn = pdTRUE;
		}
	} else {
		stats.evnt_que_err_cnt++;
	}
}

/**
 * set_udp_addr
 */
void set_udp_addr(int adr)
{
        enum udp_state us = udp_state;

	if (udp_state == UDP_STATE_DEFAULT) {
		if (adr != 0) {
			udp_state = UDP_STATE_ADDRESSED;
                        UDP->UDP_FADDR = UDP_FADDR_FEN | UDP_FADDR_FADD(adr);
			UDP->UDP_GLB_STAT |= UDP_GLB_STAT_FADDEN;
		}
	} else if (udp_state == UDP_STATE_ADDRESSED) {
		if (adr == 0) {
			udp_state = UDP_STATE_DEFAULT;
                        UDP->UDP_FADDR = UDP_FADDR_FEN | UDP_FADDR_FADD(0);
                        UDP->UDP_GLB_STAT &= ~UDP_GLB_STAT_FADDEN;
		} else {
			UDP->UDP_FADDR = UDP_FADDR_FEN | UDP_FADDR_FADD(adr);
		}
	}
#if UDP_LOG_STATE_EVENTS == 1
	if (us != udp_state) {
		log_state_event("");
	}
#endif
	if (us != udp_state) {
		notif_state_event();
	}
}

/**
 * set_udp_confg
 */
void set_udp_confg(boolean_t conf)
{
	if (conf == TRUE) {
		udp_state = UDP_STATE_CONFIGURED;
		UDP->UDP_GLB_STAT |= UDP_GLB_STAT_CONFG;
	} else {
		udp_state = UDP_STATE_ADDRESSED;
		UDP->UDP_GLB_STAT &= ~UDP_GLB_STAT_CONFG;
	}
#if UDP_LOG_STATE_EVENTS == 1
	log_state_event("");
#endif
	notif_state_event();
}

/**
 * get_udp_state
 */
enum udp_state get_udp_state(void)
{
	return (udp_state);
}

/**
 * is_udp_endp_enabled
 */
boolean_t is_udp_endp_enabled(int epn)
{
	if (endp_arry[epn].state == EP_DISABLED_STATE) {
                return (FALSE);
        } else {
		return (TRUE);
	}
}

/**
 * get_udp_endp_dir
 */
enum udp_endp_dir get_udp_endp_dir(int epn)
{
	if (((UDP->UDP_CSR[epn] & UDP_CSR_EPTYPE_Msk) >> UDP_CSR_EPTYPE_Pos) < 5) {
		return (UDP_ENDP_DIR_OUT);
	} else {
        	return (UDP_ENDP_DIR_IN);
	}
}

/**
 * is_udp_endp_halted
 */
boolean_t is_udp_endp_halted(int epn)
{
	if (endp_arry[epn].state == EP_HALTED_STATE) {
                return (TRUE);
        } else {
		return (FALSE);
	}
}

/**
 * set_rmt_wkup_feat
 */
void set_rmt_wkup_feat(boolean_t b)
{
	if ((rmt_wkup_feat = b)) {
		UDP->UDP_GLB_STAT |= UDP_GLB_STAT_RMWUPE;
	} else {
		UDP->UDP_GLB_STAT &= ~UDP_GLB_STAT_RMWUPE;
	}
}

/**
 * get_rmt_wkup_feat
 */
boolean_t get_rmt_wkup_feat(void)
{
	return (rmt_wkup_feat);
}

/**
 * add_udp_sof_intr_clbk
 */
void add_udp_sof_intr_clbk(void (*clbk)(uint32_t))
{
	sof_intr_clbk = clbk;
}

/**
 * add_udp_endp0_rxstp_clbk
 */
void add_udp_endp0_rxstp_clbk(void (*clbk)(void))
{
	endp0_rxstp_clbk = clbk;
}

/**
 * add_udp_endp0_txcomp_clbk
 */
void add_udp_endp0_txcomp_clbk(void (*clbk)(void))
{
	endp0_txcomp_clbk = clbk;
}

/**
 * add_udp_endp0_stlsnt_clbk
 */
void add_udp_endp0_stlsnt_clbk(void (*clbk)(void))
{
	endp0_stlsnt_clbk = clbk;
}

/**
 * add_udp_endp0_rxdata_clbk
 */
void add_udp_endp0_rxdata_clbk(void (*clbk)(int nmb))
{
	endp0_rxdata_clbk = clbk;
}

/**
 * read_udp_endp0_fifo
 */
void read_udp_endp0_fifo(void *bf, int nmb)
{
	for (int i = 0; i < nmb; i++) {
                *((uint8_t *) bf + i) = UDP->UDP_FDR[0];
	}
}

/**
 * write_udp_endp0_fifo
 */
void write_udp_endp0_fifo(const uint8_t *bf, int nmb)
{
	if (UDP->UDP_CSR[0] & UDP_CSR_TXPKTRDY) {
		stats.fifo_nrdy_cnt++;
	}
	for (int i = 0; i < nmb; i++) {
                UDP->UDP_FDR[0] = *(bf + i);
	}
}

/**
 * udp_endp0_rxstp_done
 */
void udp_endp0_rxstp_done(enum udp_ctl_trans_dir dir)
{
	if (dir == UDP_CTL_TRANS_OUT) {
		clr_csr(0, UDP_CSR_DIR);
	} else {
		set_csr(0, UDP_CSR_DIR);
	}
	clr_csr(0, UDP_CSR_RXSETUP);
}

/**
 * udp_endp0_rxdata_done
 */
void udp_endp0_rxdata_done(void)
{
	clr_csr(0, UDP_CSR_RX_DATA_BK0);
}

/**
 * udp_endp0_txcomp_accept
 */
void udp_endp0_txcomp_accept(void)
{
	clr_csr(0, UDP_CSR_TXCOMP);
}

/**
 * udp_endp0_stlsnt_accept
 */
void udp_endp0_stlsnt_accept(void)
{
	clr_csr(0, UDP_CSR_STALLSENT);
}

/**
 * udp_endp0_tx_pkt_rdy
 */
void udp_endp0_tx_pkt_rdy(void)
{
	set_csr(0, UDP_CSR_TXPKTRDY);
	clr_csr(0, UDP_CSR_TXCOMP);
}

/**
 * udp_endp0_req_stl
 */
void udp_endp0_req_stl(void)
{
	set_csr(0, UDP_CSR_FORCESTALL);
}

/**
 * udp_endp0_disable_stl
 */
void udp_endp0_disable_stl(void)
{
	clr_csr(0, UDP_CSR_FORCESTALL);
        clr_csr(0, UDP_CSR_STALLSENT);
}

/**
 * udp_endp0_pkt_sz
 */
int udp_endp0_pkt_sz(void)
{
	return (endp_pkt_size(0));
}

/**
 * set_udp_task_wkn
 */
void set_udp_task_wkn(void)
{
	task_wkn = pdTRUE;
}

/**
 * udp_pullup_on
 */
void udp_pullup_on(void)
{
	UDP->UDP_TXVC |= UDP_TXVC_PUON;
}

/**
 * udp_pullup_off
 */
void udp_pullup_off(void)
{
	UDP->UDP_TXVC &= ~UDP_TXVC_PUON;
}

/**
 * get_udp_stats
 */
struct udp_stats *get_udp_stats(void)
{
	return (&stats);
}

#if TERMOUT == 1
/**
 * log_udp_stats
 */
void log_udp_stats(void)
{
	if (stats.bad_csr_flags_isr_cnt) {
		msg(INF, "udp.c: bad_csr_flags_isr=%hu\n", stats.bad_csr_flags_isr_cnt);
	}
	if (stats.set_csr_tmo_cnt) {
		msg(INF, "udp.c: set_csr_tmo=%hu\n", stats.set_csr_tmo_cnt);
	}
	if (stats.clr_csr_tmo_cnt) {
		msg(INF, "udp.c: clr_csr_tmo=%hu\n", stats.clr_csr_tmo_cnt);
	}
	if (stats.data_bkx_hwerr_cnt) {
		msg(INF, "udp.c: data_bkx_hwerr=%hu\n", stats.data_bkx_hwerr_cnt);
	}
	if (stats.fifo_nrdy_cnt) {
		msg(INF, "udp.c: fifo_nrdy=%hu\n", stats.fifo_nrdy_cnt);
	}
	if (stats.unexp_endp_enab_cnt) {
		msg(INF, "udp.c: unexp_endp_enab=%hu\n", stats.unexp_endp_enab_cnt);
	}
	if (stats.unexp_endp_hlt_cnt) {
		msg(INF, "udp.c: unexp_endp_hlt=%hu\n", stats.unexp_endp_hlt_cnt);
	}
	if (stats.unexp_endp_unhlt_cnt) {
		msg(INF, "udp.c: unexp_endp_unhlt=%hu\n", stats.unexp_endp_unhlt_cnt);
	}
	if (stats.unexp_rxstp_cnt) {
		msg(INF, "udp.c: unexp_rxstp=%hu\n", stats.unexp_rxstp_cnt);
	}
	if (stats.unexp_stls_cnt) {
		msg(INF, "udp.c: unexp_stls=%hu\n", stats.unexp_stls_cnt);
	}
	if (stats.unexp_wake_cnt) {
		msg(INF, "udp.c: unexp_wake=%hu\n", stats.unexp_wake_cnt);
	}
	if (stats.rx_on_inact_ep_cnt) {
		msg(INF, "udp.c: rx_on_inact_ep=%hu\n", stats.rx_on_inact_ep_cnt);
	}
	if (stats.rx_on_in_ep_cnt) {
		msg(INF, "udp.c: rx_on_in_ep=%hu\n", stats.rx_on_in_ep_cnt);
	}
	if (stats.txc_on_inact_ep_cnt) {
		msg(INF, "udp.c: txc_on_inact_ep=%hu\n", stats.txc_on_inact_ep_cnt);
	}
	if (stats.txc_on_out_ep_cnt) {
		msg(INF, "udp.c: txc_on_out_ep=%hu\n", stats.txc_on_out_ep_cnt);
	}
	if (stats.bad_pkt_sz_cnt) {
		msg(INF, "udp.c: bad_pkt_sz=%hu\n", stats.bad_pkt_sz_cnt);
	}
	if (stats.ep_dise_cnt) {
		msg(INF, "udp.c: ep_rste=%hu\n", stats.ep_dise_cnt);
	}
	if (stats.isr_que_err_cnt) {
		msg(INF, "udp.c: isr_que_err=%hu\n", stats.isr_que_err_cnt);
	}
	if (stats.evnt_que_err_cnt) {
		msg(INF, "udp.c: evnt_que_err=%hu\n", stats.evnt_que_err_cnt);
	}
	if (stats.nfstal_cnt) {
		msg(INF, "udp.c: nfstal=%hu\n", stats.nfstal_cnt);
	}
	if (stats.unexp_ep_rst_cnt) {
		msg(INF, "udp.c: unexp_ep_rst=%hu\n", stats.unexp_ep_rst_cnt);
	}
	if (stats.brst_cnt) {
		msg(INF, "udp.c: brst=%hu\n", stats.brst_cnt);
	}
	if (stats.susp_cnt) {
		msg(INF, "udp.c: susp=%hu\n", stats.susp_cnt);
	}
	if (stats.wake_cnt) {
		msg(INF, "udp.c: wake=%hu\n", stats.wake_cnt);
	}
	int log_stls = UDP_EP_NMB;
	for (int i = 0; i < UDP_EP_NMB; i++) {
		if (stats.stls_cnt[i]) {
			log_stls = i;
		}
	}
	if (log_stls != UDP_EP_NMB) {
		UBaseType_t pr = uxTaskPriorityGet(NULL);
                vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
		msg(INF, "udp.c: stls ");
		for (int i = 0; i < UDP_EP_NMB; i++) {
			if (stats.stls_cnt[i]) {
				if (log_stls != i) {
					msg(INF, "e%d=%hu ", i, stats.stls_cnt[i]);
				} else {
					msg(INF, "e%d=%hu\n", i, stats.stls_cnt[i]);
				}
			}
		}
                vTaskPrioritySet(NULL, pr);
	}
        msg(INF, "udp.c: intr=%u\n", stats.intr_cnt);
}

/**
 * log_ep_state
 */
void log_ep_state(void)
{
	msg(INF, "udp.c: ep1=%s ep2=%s ep3=%s ep4=%s ep5=%s ep6=%s ep7=%s\n",
	    eps(1), eps(2), eps(3), eps(4), eps(5), eps(6), eps(7));
}

/**
 * eps
 */
static const char *eps(int ep)
{
	switch (endp_arry[ep].state) {
	case EP_DISABLED_STATE :
		return ("d");
	case EP_HALTED_STATE   :
		return ("h");
	case EP_IDLE_STATE     :
		return ("i");
	case EP_ACTIVE_STATE   :
		return ("a");
	case EP_SUSP_STATE   :
		return ("s");
	default :
		return ("");
	}
}

/**
 * log_csr_err
 */
void log_csr_err(void)
{
	if (stats.set_csr_tmo_cnt) {
		msg(INF, "udp.c: set_csr_err r=%d ep=%d csr=0x%.8X sb=0x%.8X u_st=%d e_st=%d\n",
		    set_csr_err.r, set_csr_err.ep, set_csr_err.csr, set_csr_err.sb, set_csr_err.u_st,
		    set_csr_err.e_st);
		msg(INF, "udp.c: set_csr_err isr=0x%.8X\n", set_csr_err.isr);
	}
	if (stats.clr_csr_tmo_cnt) {
		msg(INF, "udp.c: clr_csr_err r=%d ep=%d csr=0x%.8X sb=0x%.8X u_st=%d e_st=%d\n",
		    clr_csr_err.r, clr_csr_err.ep, clr_csr_err.csr, clr_csr_err.sb, clr_csr_err.u_st,
		    clr_csr_err.e_st);
		msg(INF, "udp.c: clr_csr_err isr=0x%.8X\n", clr_csr_err.isr);
	}
}
#endif

#if UDP_LOG_INTR_EVENTS == 1
/**
 * fmt_intr_event
 */
static void fmt_intr_event(struct udp_intr_event *p);
static void fmt_intr_event(struct udp_intr_event *p)
{
	msg(INF, "udp.c: (i)%s%s\n", p->intr, (p->ep_csr) ? p->ep_csr : "");
}

static struct udp_intr_event in_ev = {
	.type = UDP_INTR_EVENT_TYPE,
	.fmt = fmt_intr_event
};

/**
 * log_intr_event
 */
static void log_intr_event(const char *intr, const char *csr)
{
	in_ev.intr = intr;
	in_ev.ep_csr = csr;
	if (pdTRUE != xQueueSendFromISR(usb_logger.que, &in_ev, &dmy)) {
		usb_logger.que_err();
	}
}
#endif

#if UDP_LOG_STATE_EVENTS == 1
static const char *state_arry[] = {
	"powered", "default", "addressed", "configured", "suspended"
};

/**
 * fmt_state_event
 */
static void fmt_state_event(struct udp_state_event *p);
static void fmt_state_event(struct udp_state_event *p)
{
	if (*(p->txt) == '\0') {
		msg(INF, "udp.c: state_%s\n", state_arry[p->state]);
	} else {
		msg(INF, "udp.c: state_%s (%s)\n", state_arry[p->state], p->txt);
	}
}

static struct udp_state_event st_ev = {
	.type = UDP_STATE_EVENT_TYPE,
	.fmt = fmt_state_event
};

/**
 * log_state_event
 */
static void log_state_event(const char *txt)
{
	st_ev.state = udp_state;
	st_ev.txt = txt;
	if (pdTRUE != xQueueSendFromISR(usb_logger.que, &st_ev, &dmy)) {
		usb_logger.que_err();
	}
}
#endif

#if UDP_LOG_ENDP_EVENTS == 1
/**
 * endp_type_str
 */
static const char *endp_type_str(enum udp_endp_type endp_type);
static const char *endp_type_str(enum udp_endp_type endp_type)
{
	switch (endp_type) {
        case UDP_ISO_OUT_ENDP  :
        	return ("iso_out");
        case UDP_BULK_OUT_ENDP :
        	return ("bulk_out");
        case UDP_INT_OUT_ENDP  :
        	return ("int_out");
        case UDP_ISO_IN_ENDP   :
        	return ("iso_in");
        case UDP_BULK_IN_ENDP  :
        	return ("bulk_in");
        case UDP_INT_IN_ENDP   :
        	return ("int_in");
	default :
        	return ("ctrl");
	}
}

/**
 * fmt_endp_event
 */
static void fmt_endp_event(struct udp_endp_event *p);
static void fmt_endp_event(struct udp_endp_event *p)
{
	if (p->txt == ep_ev_ena_str) {
		msg(INF, "udp.c: ep%hhd[%s,%db,%d] %s\n",
		    p->num, endp_type_str(p->endp_type),
		    endp_banks_nmb(p->num),
		    endp_pkt_size(p->num),
		    p->txt);
	} else {
		msg(INF, "udp.c: ep%hhd %s\n", p->num, p->txt);
	}
}

static struct udp_endp_event ep_ev = {
	.type = UDP_ENDP_EVENT_TYPE,
	.fmt = fmt_endp_event
};

/**
 * log_endp_event
 */
static void log_endp_event(int epn, const char *txt)
{
	ep_ev.endp_type = (UDP->UDP_CSR[epn] & UDP_CSR_EPTYPE_Msk) >> UDP_CSR_EPTYPE_Pos;
	ep_ev.num = epn;
	ep_ev.txt = txt;
	if (pdTRUE != xQueueSendFromISR(usb_logger.que, &ep_ev, &dmy)) {
		usb_logger.que_err();
	}
}
#endif

#if UDP_LOG_OUT_IRP_EVENTS == 1
/**
 * fmt_out_irp_event
 */
static void fmt_out_irp_event(struct udp_out_irp_event *p);
static void fmt_out_irp_event(struct udp_out_irp_event *p)
{
	msg(INF, "udp.c: <irp[%hhd]b%hhd %hd(%hd)%s%s\n",
	    p->epn, p->bnk, p->sz, p->cnt,
	    (p->txt) ? " sig=" : "", (p->txt) ? p->txt : "");
}

static struct udp_out_irp_event oirp_ev = {
	.type = UDP_OUT_IRP_EVENT_TYPE,
	.fmt = fmt_out_irp_event
};

/**
 * log_out_irp_event
 */
static void log_out_irp_event(int epn, unsigned int bnk, int sz, int cnt, const char *txt)
{
	oirp_ev.epn = epn;
	oirp_ev.bnk = (bnk == UDP_CSR_RX_DATA_BK0) ? 0 : 1;
	oirp_ev.sz = sz;
        oirp_ev.cnt = cnt;
	oirp_ev.txt = txt;
	if (pdTRUE != xQueueSendFromISR(usb_logger.que, &oirp_ev, &dmy)) {
		usb_logger.que_err();
	}
}
#endif

#if UDP_LOG_ERR_EVENTS == 1
/**
 * fmt_err_event
 */
static void fmt_err_event(struct udp_err_event *p);
static void fmt_err_event(struct udp_err_event *p)
{
	msg(INF, "udp.c: (err)%s %s %s\n", p->intr, p->err, p->txt);
}

static struct udp_err_event er_ev = {
	.type = UDP_ERR_EVENT_TYPE,
	.fmt = fmt_err_event
};

/**
 * log_err_event
 */
static void log_err_event(const char *intr, const char *err, const char *txt)
{
	er_ev.intr = intr;
	er_ev.err = err;
	er_ev.txt = txt;
	if (pdTRUE != xQueueSendFromISR(usb_logger.que, &er_ev, &dmy)) {
		usb_logger.que_err();
	}
}

/**
 * log_err_event_nisr
 */
static void log_err_event_nisr(const char *intr, const char *err, const char *txt)
{
	er_ev.intr = intr;
	er_ev.err = err;
	er_ev.txt = txt;
	if (pdTRUE != xQueueSend(usb_logger.que, &er_ev, 0)) {
		usb_logger.que_err();
	}
}
#endif
