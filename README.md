
# sam-udp

The C library **sam-udp** provides an API for controlling the USB Device Port peripheral of the microcontroller.
The supported devices include microcontrollers from the Microchip (Atmel) **AT91** family, specifically the ***SAM3S*** and ***SAM4S*** chips.

### Library features

- Standardized API (for the AZTech framework).
- Handling of low-level USB device port events and states (the USB Device Port interrupt).
- Communication through USB I/O request packet functions (the udp\_in\_irp(), udp\_out\_irp() functions).
- Designed for real-time multitasking applications (dependent on FreeRTOS).
- Extended debugging features.
