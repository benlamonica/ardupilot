/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Ben La Monica
 */

#include "UARTDriver.h"

#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/uart.h>
#include <pico/stdio_usb.h>
#include <tusb.h>

using namespace RP2350;

#ifndef HAL_RP2350_UART_TX_BUFSZ
#define HAL_RP2350_UART_TX_BUFSZ 1024
#endif

#ifndef HAL_RP2350_UART_RX_BUFSZ
#define HAL_RP2350_UART_RX_BUFSZ 512
#endif

#ifndef HAL_RP2350_UART_DEVICES
// a board may declare no RP2350_SERIAL ports at all; the table still has to
// exist for the code below to compile, and a count of zero leaves every
// hardware index invalid
#define HAL_RP2350_UART_DEVICES { 0, 0, 0 }
#define HAL_RP2350_NUM_UARTS 0
#endif

static const UARTDesc uart_desc[] = { HAL_RP2350_UART_DEVICES };

#ifndef HAL_RP2350_NUM_UARTS
#define HAL_RP2350_NUM_UARTS ARRAY_SIZE(uart_desc)
#endif

// the SDK's interrupt handlers take no argument, so the driver that owns each
// hardware UART is recorded here for them to find
static UARTDriver *irq_owner[2];

static void uart0_irq_handler()
{
    if (irq_owner[0] != nullptr) {
        irq_owner[0]->_rx_irq();
    }
}

static void uart1_irq_handler()
{
    if (irq_owner[1] != nullptr) {
        irq_owner[1]->_rx_irq();
    }
}

/*
  The USB console deliberately bypasses Pico-SDK's stdio_put_string()/printf()
  path: that layer does LF->CRLF translation (PICO_STDIO_ENABLE_CRLF_SUPPORT),
  which would corrupt any 0x0A byte inside a binary MAVLink frame. Talking to
  TinyUSB's CDC API directly keeps writes byte-for-byte. stdio_usb_init() is
  still reused for the USB device init/descriptor/IRQ plumbing it provides.
  The hardware UARTs write the data register directly for the same reason,
  rather than going through uart_putc().
*/

void UARTDriver::_begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace)
{
    if (!_writebuf.set_size(txSpace ? txSpace : HAL_RP2350_UART_TX_BUFSZ)) {
        return;
    }

    if (is_usb()) {
        if (!_initialized) {
            _initialized = stdio_usb_init();
        }
        return;
    }

    // an index this board's hwdef.dat does not declare leaves the port
    // unusable, rather than driving whichever pins happen to follow the table
    if (_uart_index >= (int8_t)HAL_RP2350_NUM_UARTS) {
        return;
    }

    const UARTDesc &desc = uart_desc[_uart_index];
    uart_inst_t *uart = uart_get_instance(desc.port);

    // stop the peripheral asserting its interrupt while _readbuf is resized.
    // This masks at the UART rather than at the NVIC because the NVIC is
    // per-core, so disabling it here would only cover whichever core this
    // call happens to run on.
    uart_set_irqs_enabled(uart, false, false);

    if (!_readbuf.set_size(rxSpace ? rxSpace : HAL_RP2350_UART_RX_BUFSZ)) {
        return;
    }

    if (!_initialized) {
        gpio_set_function(desc.tx, UART_FUNCSEL_NUM(uart, desc.tx));
        gpio_set_function(desc.rx, UART_FUNCSEL_NUM(uart, desc.rx));
        uart_init(uart, baud);

        // Both the handler and the NVIC enable apply to the calling core
        // only, so they are done together and once: RX is then serviced on
        // whichever core first called begin(), which is the main thread.
        irq_owner[desc.port] = this;
        const uint irq = UART0_IRQ + desc.port;
        irq_set_exclusive_handler(irq, desc.port == 0 ? uart0_irq_handler : uart1_irq_handler);
        irq_set_enabled(irq, true);
        _initialized = true;
    } else {
        // a re-begin() only ever changes the baudrate; leave the pin muxing
        // and the interrupt wiring alone
        uart_set_baudrate(uart, baud);
    }

    _baudrate = baud;

    // RX only. TX is polled instead, see drain_writebuf().
    uart_set_irqs_enabled(uart, true, false);
}

void UARTDriver::_end()
{
    if (!is_usb() && _initialized) {
        uart_inst_t *uart = uart_get_instance(uart_desc[_uart_index].port);
        uart_set_irqs_enabled(uart, false, false);
        uart_deinit(uart);
    }
    _initialized = false;
    _writebuf.set_size(0);
    _readbuf.set_size(0);
}

/*
  RX interrupt: move everything the FIFO holds into _readbuf. This is the sole
  producer for that buffer, which is what makes the lock-free ByteBuffer safe
  to use here.
*/
void UARTDriver::_rx_irq()
{
    uart_inst_t *uart = uart_get_instance(uart_desc[_uart_index].port);
    while (uart_is_readable(uart)) {
        // the FIFO has to be emptied even when _readbuf is full, or the
        // interrupt would never clear; excess bytes are dropped
        const uint8_t c = (uint8_t)uart_get_hw(uart)->dr;
        _readbuf.write(&c, 1);
    }
}

/*
  push queued bytes into the hardware, as far as it has drained. Anything that
  does not fit stays queued for the next call.
*/
void UARTDriver::drain_writebuf()
{
    if (!_initialized) {
        return;
    }
    WITH_SEMAPHORE(_write_sem);

    if (is_usb()) {
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
        return;
    }

    /*
      TX is polled rather than interrupt driven. The PL011 only raises its TX
      interrupt as the FIFO drains past the threshold, so it never fires from
      an empty FIFO and an interrupt-driven path would still have to prime it
      from here - which would give _writebuf a second consumer. Polling keeps
      the interrupt as the RX producer only. The UART thread calls this at
      1kHz and _write() calls it directly, so a burst leaves at FIFO speed and
      a sustained stream is limited to the 32-byte FIFO per tick.
    */
    uart_inst_t *uart = uart_get_instance(uart_desc[_uart_index].port);
    uint8_t c;
    while (uart_is_writable(uart) && _writebuf.read(&c, 1) == 1) {
        uart_get_hw(uart)->dr = c;
    }
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
    return is_usb() ? tud_cdc_available() : _readbuf.available();
}

bool UARTDriver::_discard_input()
{
    if (!_initialized) {
        return false;
    }
    uint8_t buf[64];
    if (is_usb()) {
        while (tud_cdc_available()) {
            tud_cdc_read(buf, sizeof(buf));
        }
    } else {
        // read the bytes out rather than clearing the buffer, so that the
        // interrupt stays the only writer of _readbuf's indices
        while (_readbuf.read(buf, sizeof(buf)) > 0) {}
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
    return is_usb() ? tud_cdc_read(buffer, size) : _readbuf.read(buffer, size);
}
