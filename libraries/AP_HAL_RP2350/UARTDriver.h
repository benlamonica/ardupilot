#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>
#include "HAL_RP2350_Namespace.h"
#include "Semaphores.h"

namespace RP2350
{

/*
  one hardware UART, as declared by an RP2350_SERIAL line in hwdef.dat; the
  generated table of these is HAL_RP2350_UART_DEVICES
*/
struct UARTDesc {
    uint8_t port;   // 0 for uart0, 1 for uart1
    uint8_t rx;     // GPIO carrying RX
    uint8_t tx;     // GPIO carrying TX
};

}

/*
  serial driver for both the USB CDC console and the hardware UARTs. The two
  share the write buffering and locking, and differ only in how bytes reach
  the wire and where received bytes are buffered.
*/
class RP2350::UARTDriver : public AP_HAL::UARTDriver
{
public:
    // index into HAL_RP2350_UART_DEVICES, or USB_CONSOLE for the USB CDC port
    static constexpr int8_t USB_CONSOLE = -1;

    UARTDriver(int8_t uart_index) : _uart_index(uart_index) {}

    bool is_initialized() override;
    bool tx_pending() override;

    uint32_t txspace() override;
    uint32_t get_baud_rate() const override { return _baudrate; }

    void _timer_tick() override;

    // called from the UART interrupt; public only because the SDK's handlers
    // take no argument and so have to reach it from a free function
    void _rx_irq();

protected:
    void _begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) override;
    size_t _write(const uint8_t *buffer, size_t size) override;
    ssize_t _read(uint8_t *buffer, uint16_t size) override WARN_IF_UNUSED;
    void _end() override;
    void _flush() override;
    uint32_t _available() override;
    bool _discard_input() override;

private:
    // push as much of _writebuf as the hardware will currently accept
    void drain_writebuf();

    bool is_usb() const { return _uart_index == USB_CONSOLE; }

    const int8_t _uart_index;
    uint32_t _baudrate;

    // AP_HAL::UARTDriver::write() does not retry a short _write(), so
    // writes are queued here and drained as the hardware consumes them
    // rather than being dropped when the FIFO is full.
    ByteBuffer _writebuf{0};

    // filled by _rx_irq(), drained by _read(). Unused for the USB console,
    // whose received bytes are already buffered inside TinyUSB.
    ByteBuffer _readbuf{0};

    // ByteBuffer is only safe for a single producer and a single consumer.
    // Threads on both cores write to a port, and the draining side runs from
    // both _write() and the UART thread, so both ends of _writebuf are
    // serialised here. _readbuf needs no lock: its only producer is the
    // interrupt, which could not take a FreeRTOS mutex in any case.
    Semaphore _write_sem;

    bool _initialized;
};
