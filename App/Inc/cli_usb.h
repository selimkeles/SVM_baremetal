#ifndef APP_INC_CLI_USB_H_
#define APP_INC_CLI_USB_H_

#include <stdint.h>
#include <stddef.h>

/* Call once from MX_FREERTOS_Init() before spawning the CLI task. */
void cli_usb_init(void);

/* Called from CDC_Receive_FS (USB IRQ context). */
void cli_usb_rx_from_isr(const uint8_t *buf, uint16_t n);

/* Block until up to n bytes are available. Returns bytes received.
 * timeout_ms == 0 returns immediately with whatever is in the buffer. */
int cli_usb_read(uint8_t *dst, size_t n, uint32_t timeout_ms);

/* Send n bytes over CDC. Task context only — never call from ISR.
 * Returns 0 on success, -1 if USB not connected, -2 if TX timed out. */
int cli_usb_write(const uint8_t *buf, uint16_t n);

/* printf-style helper. Uses a 128-byte stack buffer; output is truncated
 * if the formatted string exceeds that. Task context only. */
int cli_printf(const char *fmt, ...);

#endif /* APP_INC_CLI_USB_H_ */
