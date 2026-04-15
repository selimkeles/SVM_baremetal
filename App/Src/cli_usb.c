#include "cli_usb.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* Stream buffer for CDC RX → CLI task. Static; created in cli_usb_init(). */
static StreamBufferHandle_t s_rx_stream;

/* Expose the USB device handle for the NULL-guard in cli_usb_write.
 * Defined in USB_DEVICE/App/usb_device.c, declared extern in usbd_cdc_if.c. */
extern USBD_HandleTypeDef hUsbDeviceFS;

void cli_usb_init(void)
{
    s_rx_stream = xStreamBufferCreate(256, 1);
    configASSERT(s_rx_stream != NULL);
}

void cli_usb_rx_from_isr(const uint8_t *buf, uint16_t n)
{
    BaseType_t woken = pdFALSE;
    xStreamBufferSendFromISR(s_rx_stream, buf, (size_t)n, &woken);
    portYIELD_FROM_ISR(woken);
}

int cli_usb_read(uint8_t *dst, size_t n, uint32_t timeout_ms)
{
    return (int)xStreamBufferReceive(s_rx_stream, dst, n,
                                     pdMS_TO_TICKS(timeout_ms));
}

int cli_usb_write(const uint8_t *buf, uint16_t n)
{
    /* Guard: CDC_Transmit_FS dereferences pClassData without a NULL check.
     * Avoid a hard fault if the host hasn't enumerated yet. */
    if (hUsbDeviceFS.pClassData == NULL) {
        return -1;
    }

    for (int tries = 0; tries < 50; ++tries) {
        uint8_t r = CDC_Transmit_FS((uint8_t *)buf, n);
        if (r == USBD_OK)   return 0;
        if (r == USBD_FAIL) return -1;   /* not connected / not configured */
        osDelay(1);                       /* USBD_BUSY — yield and retry */
    }
    return -2;  /* TX endpoint stuck */
}

int cli_printf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return n;
    if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;  /* truncated */
    return cli_usb_write((const uint8_t *)buf, (uint16_t)n);
}
