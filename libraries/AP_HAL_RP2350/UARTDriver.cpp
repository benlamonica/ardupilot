#include "UARTDriver.h"

#include <pico/stdio_usb.h>
#include <tusb.h>

using namespace RP2350;

#ifndef HAL_RP2350_UART_TX_BUFSZ
#define HAL_RP2350_UART_TX_BUFSZ 1024
#endif

/*
  This deliberately bypasses Pico-SDK's stdio_put_string()/printf() path:
  that layer does LF->CRLF translation (PICO_STDIO_ENABLE_CRLF_SUPPORT),
  which would corrupt any 0x0A byte inside a binary MAVLink frame. Talking
  to TinyUSB's CDC API directly keeps writes byte-for-byte. stdio_usb_init()
  is still reused for the USB device init/descriptor/IRQ-driven tud_task()
  plumbing it already provides.
*/

void UARTDriver::_begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace)
{
    if (!_writebuf.set_size(txSpace ? txSpace : HAL_RP2350_UART_TX_BUFSZ)) {
        return;
    }
    if (!_initialized) {
        _initialized = stdio_usb_init();
    }
}

void UARTDriver::_end()
{
    _initialized = false;
    _writebuf.set_size(0);
}

/*
  push queued bytes into the CDC FIFO, as far as the host has drained it.
  Anything that does not fit stays queued for the next call.
*/
void UARTDriver::drain_writebuf()
{
    if (!_initialized) {
        return;
    }
    WITH_SEMAPHORE(_write_sem);
    while (_writebuf.available() > 0) {
        uint32_t avail = tud_cdc_write_available();
        if (avail == 0) {
            break;
        }
        uint8_t tmp[64];
        uint32_t n = sizeof(tmp);
        if (avail < n) {
            n = avail;
        }
        if (_writebuf.available() < n) {
            n = _writebuf.available();
        }
        n = _writebuf.read(tmp, n);
        if (n == 0) {
            break;
        }
        tud_cdc_write(tmp, n);
    }
    tud_cdc_write_flush();
}

void UARTDriver::_timer_tick()
{
    drain_writebuf();
}

void UARTDriver::_flush()
{
    drain_writebuf();
}

bool UARTDriver::is_initialized()
{
    return _initialized;
}

bool UARTDriver::tx_pending()
{
    return _writebuf.available() > 0;
}

uint32_t UARTDriver::txspace()
{
    return _writebuf.space();
}

uint32_t UARTDriver::_available()
{
    if (!_initialized) {
        return 0;
    }
    return tud_cdc_available();
}

bool UARTDriver::_discard_input()
{
    if (!_initialized) {
        return false;
    }
    uint8_t buf[64];
    while (tud_cdc_available()) {
        tud_cdc_read(buf, sizeof(buf));
    }
    return true;
}

size_t UARTDriver::_write(const uint8_t *buffer, size_t size)
{
    if (!_initialized) {
        return 0;
    }
    size_t ret;
    {
        // keep each write contiguous in the buffer, so concurrent writers on
        // either core cannot interleave within a line
        WITH_SEMAPHORE(_write_sem);
        ret = _writebuf.write(buffer, size);
    }
    drain_writebuf();
    return ret;
}

ssize_t UARTDriver::_read(uint8_t *buffer, uint16_t size)
{
    if (!_initialized) {
        return 0;
    }
    return tud_cdc_read(buffer, size);
}
