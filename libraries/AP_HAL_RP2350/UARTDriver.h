#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>
#include "HAL_RP2350_Namespace.h"

/*
  Phase 1 console driver: routes hal.console through Pico-SDK's USB CDC
  stdio driver (pico_stdio_usb). GPS/telemetry-grade UARTDriver support for
  the other serial ports is Phase 2 work (see the RP2350 port plan).
*/
class RP2350::UARTDriver : public AP_HAL::UARTDriver
{
public:
    bool is_initialized() override;
    bool tx_pending() override;

    uint32_t txspace() override;

    void _timer_tick() override;

protected:
    void _begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) override;
    size_t _write(const uint8_t *buffer, size_t size) override;
    ssize_t _read(uint8_t *buffer, uint16_t size) override WARN_IF_UNUSED;
    void _end() override;
    void _flush() override;
    uint32_t _available() override;
    bool _discard_input() override;

private:
    // drain as much of _writebuf into the USB CDC FIFO as it will accept
    void drain_writebuf();

    // AP_HAL::UARTDriver::write() does not retry a short _write(), so
    // writes are queued here and drained as the host consumes them rather
    // than being dropped when the CDC FIFO is full.
    ByteBuffer _writebuf{0};

    bool _initialized = false;
};
