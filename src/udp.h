/*
 * udp.h
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

#ifndef UDP_H
#define UDP_H

#include <logger.h>

#define UDP_EP_NMB 8

enum udp_state {
	UDP_STATE_POWERED,
	UDP_STATE_DEFAULT,
	UDP_STATE_ADDRESSED,
	UDP_STATE_CONFIGURED,
        UDP_STATE_SUSPENDED
};

enum udp_endp_type {
	UDP_CTRL_ENDP,
        UDP_ISO_OUT_ENDP,
        UDP_BULK_OUT_ENDP,
        UDP_INT_OUT_ENDP,
        UDP_ISO_IN_ENDP = 5,
        UDP_BULK_IN_ENDP,
        UDP_INT_IN_ENDP
};

enum udp_endp_dir {
	UDP_ENDP_DIR_OUT,
        UDP_ENDP_DIR_IN
};

#define UDP_INTR_EVENT_TYPE 0

struct udp_intr_event {
	int8_t type;
	const char *intr;
	const char *ep_csr;
	void (*fmt)(struct udp_intr_event *);
};

#define UDP_STATE_EVENT_TYPE 1

struct udp_state_event {
	int8_t type;
	enum udp_state state;
	const char *txt;
        void (*fmt)(struct udp_state_event *);
};

#define UDP_ENDP_EVENT_TYPE 2

struct udp_endp_event {
	int8_t type;
	enum udp_endp_type endp_type;
	int8_t num;
	const char *txt;
        void (*fmt)(struct udp_endp_event *);
};

#define UDP_OUT_IRP_EVENT_TYPE 3

struct udp_out_irp_event {
	int8_t type;
        int8_t epn;
        int8_t bnk;
	short sz;
	short cnt;
        const char *txt;
	void (*fmt)(struct udp_out_irp_event *);
};

#define UDP_ERR_EVENT_TYPE 4

struct udp_err_event {
	int8_t type;
	const char *intr;
	const char *err;
        const char *txt;
	void (*fmt)(struct udp_err_event *);
};

struct udp_stats {
	unsigned short bad_csr_flags_isr_cnt;
	unsigned short set_csr_tmo_cnt;
        unsigned short clr_csr_tmo_cnt;
        unsigned short data_bkx_hwerr_cnt;
	unsigned short fifo_nrdy_cnt;
	unsigned short unexp_endp_enab_cnt;
	unsigned short unexp_endp_hlt_cnt;
	unsigned short unexp_endp_unhlt_cnt;
        unsigned short unexp_rxstp_cnt;
        unsigned short unexp_stls_cnt;
	unsigned short unexp_wake_cnt;
	unsigned short rx_on_inact_ep_cnt;
	unsigned short rx_on_in_ep_cnt;
	unsigned short txc_on_inact_ep_cnt;
        unsigned short txc_on_out_ep_cnt;
        unsigned short bad_pkt_sz_cnt;
	unsigned short ep_dise_cnt;
	unsigned short isr_que_err_cnt;
	unsigned short evnt_que_err_cnt;
	unsigned short nfstal_cnt;
	unsigned short unexp_ep_rst_cnt;
	unsigned short brst_cnt;
	unsigned short susp_cnt;
	unsigned short wake_cnt;
	unsigned short stls_cnt[UDP_EP_NMB];
        unsigned int intr_cnt;
};

enum udp_ctl_trans_dir {
	UDP_CTL_TRANS_OUT,
        UDP_CTL_TRANS_IN
};

/**
 * init_udp
 */
void init_udp(logger_t *logger);

/**
 * get_udp_evnt_que
 */
QueueHandle_t get_udp_evnt_que(void);

#if configUSE_QUEUE_SETS == 1
/**
 * add_udp_evnt_que_to_qset
 */
void add_udp_evnt_que_to_qset(QueueSetHandle_t qset);
#endif

/**
 * udp_in_irp
 */
int udp_in_irp(int epn, void *buf, int size, boolean_t use_zero_pkt);

/**
 * udp_out_irp
 */
int udp_out_irp(int epn, void *buf, int size, int min_rcv_nmb, int *rcv_nmb);

/**
 * init_udp_endp_que
 */
void init_udp_endp_que(int epn);

/**
 * enable_udp_endp
 */
void enable_udp_endp(int epn, enum udp_endp_type etp);

/**
 * disable_udp_endp
 */
void disable_udp_endp(int epn);

/**
 * halt_udp_endp
 */
void halt_udp_endp(int epn);

/**
 * un_halt_udp_endp
 */
void un_halt_udp_endp(int epn);

/**
 * set_udp_addr
 */
void set_udp_addr(int adr);

/**
 * set_udp_confg
 */
void set_udp_confg(boolean_t conf);

/**
 * get_udp_state
 */
enum udp_state get_udp_state(void);

/**
 * is_udp_endp_enabled
 */
boolean_t is_udp_endp_enabled(int epn);

/**
 * get_udp_endp_dir
 */
enum udp_endp_dir get_udp_endp_dir(int epn);

/**
 * is_udp_endp_halted
 */
boolean_t is_udp_endp_halted(int epn);

/**
 * set_rmt_wkup_feat
 */
void set_rmt_wkup_feat(boolean_t b);

/**
 * get_rmt_wkup_feat
 */
boolean_t get_rmt_wkup_feat(void);

/**
 * add_udp_sof_intr_clbk
 */
void add_udp_sof_intr_clbk(void (*clbk)(uint32_t));

/**
 * add_udp_endp0_rxstp_clbk
 */
void add_udp_endp0_rxstp_clbk(void (*clbk)(void));

/**
 * add_udp_endp0_txcomp_clbk
 */
void add_udp_endp0_txcomp_clbk(void (*clbk)(void));

/**
 * add_udp_endp0_stlsnt_clbk
 */
void add_udp_endp0_stlsnt_clbk(void (*clbk)(void));

/**
 * add_udp_endp0_rxdata_clbk
 */
void add_udp_endp0_rxdata_clbk(void (*clbk)(int nmb));

/**
 * read_udp_endp0_fifo
 */
void read_udp_endp0_fifo(void *bf, int nmb);

/**
 * write_udp_endp0_fifo
 */
void write_udp_endp0_fifo(const uint8_t *bf, int nmb);

/**
 * udp_endp0_rxstp_done
 */
void udp_endp0_rxstp_done(enum udp_ctl_trans_dir dir);

/**
 * udp_endp0_rxdata_done
 */
void udp_endp0_rxdata_done(void);

/**
 * udp_endp0_txcomp_accept
 */
void udp_endp0_txcomp_accept(void);

/**
 * udp_endp0_stlsnt_accept
 */
void udp_endp0_stlsnt_accept(void);

/**
 * udp_endp0_tx_pkt_rdy
 */
void udp_endp0_tx_pkt_rdy(void);

/**
 * udp_endp0_req_stl
 */
void udp_endp0_req_stl(void);

/**
 * udp_endp0_disable_stl
 */
void udp_endp0_disable_stl(void);

/**
 * udp_endp0_pkt_sz
 */
int udp_endp0_pkt_sz(void);

/**
 * set_udp_task_wkn
 */
void set_udp_task_wkn(void);

/**
 * udp_pullup_on
 */
void udp_pullup_on(void);

/**
 * udp_pullup_off
 */
void udp_pullup_off(void);

/**
 * get_udp_stats
 */
struct udp_stats *get_udp_stats(void);

#if TERMOUT == 1
/**
 * log_udp_stats
 */
void log_udp_stats(void);

/**
 * log_ep_state
 */
void log_ep_state(void);

/**
 * log_set_csr_err
 */
void log_csr_err(void);
#endif

#endif
