#include "UARTDriver.h"

#include <pico/stdio_usb.h>
#include <tusb.h>

using namespace RP2350;

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
    if (!_initialized) {
        _initialized = stdio_usb_init();
    }
}

void UARTDriver::_end()
{
    _initialized = false;
}

void UARTDriver::_flush()
{
    tud_cdc_write_flush();
}

bool UARTDriver::is_initialized()
{
    return _initialized;
}

bool UARTDriver::tx_pending()
{
    return false;
}

uint32_t UARTDriver::txspace()
{
    if (!_initialized) {
        return 0;
    }
    return tud_cdc_write_available();
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
    uint32_t avail = tud_cdc_write_available();
    if (avail == 0) {
        return 0;
    }
    uint32_t n = size < avail ? size : avail;
    uint32_t written = tud_cdc_write(buffer, n);
    tud_cdc_write_flush();
    return written;
}

ssize_t UARTDriver::_read(uint8_t *buffer, uint16_t size)
{
    if (!_initialized) {
        return 0;
    }
    return tud_cdc_read(buffer, size);
}
