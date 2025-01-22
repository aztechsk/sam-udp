
# sam-udp

The C library **sam-udp** provides an API for controlling the USB Device Port peripheral
of the microcontroller. Supported devices include microcontrollers from the Microchip
(Atmel) **AT91** family, specifically the ***SAM3S*** and ***SAM4S*** chips.

### Library Features

- Standardized API (for the AZTech framework).
- Handling of low-level USB device port events and states (USB Device Port interrupt).
- Communication through USB I/O request packet functions (e.g., `udp_in_irp()`, `udp_out_irp()`).
- Designed for real-time multitasking applications (dependent on FreeRTOS).
- Extended debugging features.
