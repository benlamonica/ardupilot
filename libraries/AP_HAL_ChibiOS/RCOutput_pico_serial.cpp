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
 */
/*
  ESC serial passthrough on the RP2350 PIO DShot outputs.

  AP_BLHeli talks to an ESC's bootloader over its signal wire as a one-wire,
  half-duplex UART: 19200 baud, 8N1, idle high. The STM32 implementation in
  RCOutput.cpp drives the bits from a timer and DMA; here the motor pins belong
  to PIO state machines, so a passthrough channel is taken off its state machine
  and driven as plain GPIO instead:

  - setup pauses every motor channel in the mask and drives each pad high as
    GPIO, the idle line. Driven, as the STM32 path and Betaflight do, because
    BLHeli_S/Bluejay enter their bootloader only after the line has read high
    for a while, and a pull-up loses to an ESC input with a pull-down.
  - writes are refused until the channels have been off DShot and idle for
    AP_RP2350_ESC_PASSTHROUGH_SETTLE_MS. Measured on Bluejay 0.19, an ESC
    left alone answered 0.37 s after DShot stopped, while ones tried early
    took up to 2.7 s.
  - transmit bit-bangs each byte against the 1 MHz TIMER0 count with the
    system locked for that byte only (about 520 us at 19200 baud).
  - only the selected line is ever released: from the end of a write, so an
    early reply is not missed, until the read finishes, when it is driven high
    again.
  - receive timestamps every edge in the GPIO interrupt and decodes each byte
    from those timestamps, so a thread that runs late does not move the sample
    points.
  - end hands every paused channel back to its state machine.

  Only allowed while disarmed. AP_BLHeli already refuses passthrough when armed.
 */

#include <hal.h>
#include "RCOutput.h"

#if defined(RP2350) && (HAL_USE_PWM == TRUE) && HAL_DSHOT_ENABLED && AP_RP2350_ESC_PASSTHROUGH_ENABLED

#include "RCOutput_pico.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/ExpandingString.h>
#include <AP_HAL/utility/EdgeSerial.h>

using namespace ChibiOS;
using AP_HAL::EdgeSerial::Edge;

extern const AP_HAL::HAL& hal;

namespace {

// RCOutput_pico channels currently taken off DShot, and their GPIOs
uint32_t paused_mask;
uint8_t paused_gpio[RCOutput_pico::MAX_CHANNELS];

// the line in use, and whether one is selected
ioline_t serial_line;
bool serial_active;
bool receiving;

// when the channels came off DShot, and whether the first write must still wait
uint32_t takeover_ms;
bool settle_pending;

// counters for @SYS/timers.txt
struct {
    uint32_t sessions;
    uint32_t bytes_tx;
    uint32_t bytes_rx;
    uint32_t framing_errors;
    uint32_t timeouts;
    uint32_t settle_refused;    // writes refused while the lines settle
    uint32_t early_starts;      // replies already started when the line was released
    uint32_t reply_gap_us;      // end of the last write to the first edge of its reply
} stats;
uint32_t tx_end_us;
bool awaiting_reply;

// bit time in 1/16 us, so 19200 baud keeps its fractional part
uint32_t bit_time_x16;

// edges seen while receiving, written by the GPIO interrupt
constexpr uint16_t EDGE_BUFFER = 256;   // power of two
Edge edges[EDGE_BUFFER];
volatile uint16_t edge_head;
uint16_t edge_tail;

inline uint32_t now_us()
{
    return TIMER0->TIMERAWL;
}

void edge_isr(void *arg)
{
    (void)arg;
    const uint32_t t = now_us();
    const uint16_t h = edge_head;
    edges[h & (EDGE_BUFFER - 1U)].t_us = t;
    edges[h & (EDGE_BUFFER - 1U)].level = palReadLine(serial_line) ? 1U : 0U;
    edge_head = h + 1U;
    if (awaiting_reply) {
        stats.reply_gap_us = t - tx_end_us;
        awaiting_reply = false;
    }
}

/*
  Start listening. Edge capture is armed before the line is released: an ESC
  may begin its reply while we still drive the end of our stop bit, and its
  start bit would then show no edge of its own. If the line is already low
  once released, that is taken as the start bit, timed from now.
 */
void start_receive()
{
    if (receiving) {
        return;
    }
    receiving = true;
    chSysLock();
    edge_tail = edge_head;
    palSetLineCallbackI(serial_line, edge_isr, nullptr);
    palEnableLineEventI(serial_line, PAL_EVENT_MODE_BOTH_EDGES);
    chSysUnlock();
    palSetLineMode(serial_line, PAL_MODE_INPUT_PULLUP);
    chSysLock();
    if (edge_head == edge_tail && palReadLine(serial_line) == PAL_LOW) {
        const uint16_t h = edge_head;
        edges[h & (EDGE_BUFFER - 1U)].t_us = AP_HAL::EdgeSerial::early_start_us(now_us(), bit_time_x16);
        edges[h & (EDGE_BUFFER - 1U)].level = 0;
        edge_head = h + 1U;
        stats.early_starts++;
        if (awaiting_reply) {
            stats.reply_gap_us = now_us() - tx_end_us;
            awaiting_reply = false;
        }
    }
    chSysUnlock();
}

void stop_receive()
{
    if (!receiving) {
        return;
    }
    receiving = false;
    chSysLock();
    palDisableLineEventI(serial_line);
    chSysUnlock();
}

// the idle state of a passthrough line: driven high
void drive_idle(ioline_t line)
{
    palSetLine(line);
    palSetLineMode(line, PAL_MODE_OUTPUT_PUSHPULL);
}

bool edge_available()
{
    return edge_tail != edge_head;
}

const Edge &peek_edge()
{
    return edges[edge_tail & (EDGE_BUFFER - 1U)];
}

void pop_edge()
{
    edge_tail++;
}

// the received edges, as AP_HAL::EdgeSerial::decode_frame() reads them
struct EdgeSource {
    bool available() const { return edge_available(); }
    const Edge &peek() const { return peek_edge(); }
    void pop() { pop_edge(); }
};

}  // namespace

bool RCOutput::serial_setup_output(uint8_t chan, uint32_t baudrate, uint32_t motor_mask)
{
    if (hal.util->get_soft_armed() || baudrate == 0 || !RCOutput_pico::is_initialised()) {
        return false;
    }
    chan -= chan_offset;
    motor_mask >>= chan_offset;
    const uint32_t bound = RCOutput_pico::channel_mask() | RCOutput_pico::paused_mask();
    if (chan >= RCOutput_pico::MAX_CHANNELS || !(bound & (1U << chan))) {
        return false;
    }

    // take every motor channel off DShot, so none of them is driven while we talk
    motor_mask |= (1U << chan);
    const bool was_paused = paused_mask != 0;
    for (uint8_t i = 0; i < RCOutput_pico::MAX_CHANNELS; i++) {
        if ((motor_mask & (1U << i)) && !(paused_mask & (1U << i))) {
            const int8_t gpio = RCOutput_pico::pause_channel(i);
            if (gpio >= 0) {
                paused_gpio[i] = (uint8_t)gpio;
                paused_mask |= (1U << i);
            }
        }
    }
    if (!(paused_mask & (1U << chan))) {
        return false;
    }
    if (!was_paused) {
        takeover_ms = AP_HAL::millis();
        settle_pending = true;
        stats.sessions++;
    }

    // leave the previous line, or this one if a read left it listening, idle high
    if (serial_active) {
        stop_receive();
        drive_idle(serial_line);
    }

    serial_line = PAL_LINE(IOPORT1, paused_gpio[chan]);
    drive_idle(serial_line);
    bit_time_x16 = AP_HAL::EdgeSerial::bit_time_x16(baudrate);
    serial_active = true;
    return true;
}

bool RCOutput::serial_write_bytes(const uint8_t *bytes, uint16_t len)
{
    if (!serial_active || hal.util->get_soft_armed()) {
        return false;
    }
    if (settle_pending) {
        /*
          Leave the lines idle until the ESCs are in their bootloader. Fail
          the write without touching the line rather than block: AP_BLHeli
          then answers the configurator with an error at once, and the
          configurator retries (esc-configurator: 10 tries 250 ms apart, each
          timing out after 1 s, so a long wait here would break it).
         */
        if (AP_HAL::millis() - takeover_ms < AP_RP2350_ESC_PASSTHROUGH_SETTLE_MS) {
            stats.settle_refused++;
            return false;
        }
        settle_pending = false;
    }
    stop_receive();

    // raise the line before driving it, so switching to output is not a start bit
    palSetLine(serial_line);
    palSetLineMode(serial_line, PAL_MODE_OUTPUT_PUSHPULL);

    for (uint16_t n = 0; n < len; n++) {
        const uint16_t frame = AP_HAL::EdgeSerial::frame_bits(bytes[n]);
        // the last stop bit is held only to its middle, where the ESC may answer
        const bool last = (n + 1U == len);
        chSysLock();
        const uint32_t t0 = now_us();
        for (uint8_t bit = 0; bit < 10; bit++) {
            if (frame & (1U << bit)) {
                palSetLine(serial_line);
            } else {
                palClearLine(serial_line);
            }
            const uint32_t due = t0 + AP_HAL::EdgeSerial::tx_bit_end_us(bit, last, bit_time_x16);
            while ((int32_t)(now_us() - due) < 0) {
            }
        }
        chSysUnlock();
    }
    stats.bytes_tx += len;

    // half duplex: let go of the line and start listening at once, so an ESC
    // that answers before serial_read_bytes() is called is not missed
    tx_end_us = now_us();
    awaiting_reply = true;
    start_receive();
    return true;
}

uint16_t RCOutput::serial_read_bytes(uint8_t *buf, uint16_t len, uint32_t timeout_us)
{
    if (!serial_active || len == 0) {
        return 0;
    }
    start_receive();

    const uint32_t start = now_us();
    uint16_t count = 0;

    while (count < len) {
        // wait for a start bit: the next falling edge
        bool have_start = false;
        uint32_t t0 = 0;
        while (!have_start) {
            while (edge_available()) {
                const Edge &e = peek_edge();
                pop_edge();
                if (e.level == 0) {
                    t0 = e.t_us;
                    have_start = true;
                    break;
                }
            }
            if (have_start) {
                break;
            }
            if (now_us() - start > timeout_us) {
                stats.timeouts++;
                stop_receive();
                drive_idle(serial_line);
                return count;
            }
            hal.scheduler->delay_microseconds(20);
        }

        // wait for the whole frame
        const uint32_t frame_end = t0 + AP_HAL::EdgeSerial::rx_frame_us(bit_time_x16);
        while ((int32_t)(now_us() - frame_end) < 0) {
            hal.scheduler->delay_microseconds(20);
        }

        // replay the edges inside the frame and sample each bit
        EdgeSource src;
        uint8_t byte;
        if (!AP_HAL::EdgeSerial::decode_frame(t0, src, bit_time_x16, byte)) {
            // no stop bit: framing error, give up on this read
            stats.framing_errors++;
            break;
        }
        buf[count++] = byte;
        stats.bytes_rx++;
    }

    // the reply is in; drive the line high again, as the STM32 path does
    stop_receive();
    drive_idle(serial_line);
    return count;
}

void RCOutput::serial_end(uint32_t chanmask)
{
    (void)chanmask;
    if (serial_active) {
        stop_receive();
        serial_active = false;
    }
    // give every paused channel back to its state machine
    for (uint8_t i = 0; i < RCOutput_pico::MAX_CHANNELS; i++) {
        if (paused_mask & (1U << i)) {
            RCOutput_pico::resume_channel(i);
        }
    }
    paused_mask = 0;
    settle_pending = false;
}

void RCOutput::serial_passthrough_info(ExpandingString &str)
{
    str.printf("ESC passthrough: sessions=%lu tx=%lu rx=%lu framing=%lu timeouts=%lu settle_refused=%lu early_starts=%lu reply_gap=%luus\n",
               (unsigned long)stats.sessions, (unsigned long)stats.bytes_tx,
               (unsigned long)stats.bytes_rx, (unsigned long)stats.framing_errors,
               (unsigned long)stats.timeouts, (unsigned long)stats.settle_refused, (unsigned long)stats.early_starts,
               (unsigned long)stats.reply_gap_us);
}

#endif  // RP2350 && HAL_DSHOT_ENABLED && AP_RP2350_ESC_PASSTHROUGH_ENABLED
