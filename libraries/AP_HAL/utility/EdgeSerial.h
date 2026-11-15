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
  Timing and decoding for a bit-banged, half-duplex 8N1 serial line, such as
  the BLHeli ESC bootloader link on a motor signal wire. Times are in
  microseconds from a free-running counter. The receive side works from
  timestamped edges, so a thread that runs late does not move the sample
  points.

  No HAL calls, so the logic can be unit tested on the host.
 */
#pragma once

#include <stdint.h>

namespace AP_HAL {
namespace EdgeSerial {

// one level change on the line: when, and the level after it
struct Edge {
    uint32_t t_us;
    uint8_t level;
};

// bit time in 1/16 us, so 19200 baud keeps its fractional part
inline uint32_t bit_time_x16(uint32_t baudrate)
{
    return (16000000UL + baudrate / 2U) / baudrate;
}

// a number of half bit periods, in microseconds
inline uint32_t half_bits_us(uint32_t half_bits, uint32_t bit_x16)
{
    return (half_bits * bit_x16 + 16U) / 32U;
}

// the ten bits of an 8N1 frame, LSB first: start 0, eight data bits, stop 1
inline uint16_t frame_bits(uint8_t byte)
{
    return (uint16_t)(byte << 1) | (1U << 9);
}

/*
  When bit (0-9) of a transmitted frame ends, from the start of the frame.
  The final stop bit of a write is held only to its middle: the receiver
  takes the stop bit there and may answer at once, as after any UART.
 */
inline uint32_t tx_bit_end_us(uint8_t bit, bool last_byte, uint32_t bit_x16)
{
    const uint32_t half_bits = (last_byte && bit == 9) ? 19U : 2U * (bit + 1U);
    return half_bits_us(half_bits, bit_x16);
}

/*
  The start time to assume for a reply whose start bit had already begun
  when the line was released, so its edge was not seen. The sender started
  somewhere in the last half bit before release (it may answer from the
  middle of our stop bit); assume a quarter bit before release, which keeps
  the error within a quarter bit either way.
 */
inline uint32_t early_start_us(uint32_t t_release, uint32_t bit_x16)
{
    return t_release - (bit_x16 + 32U) / 64U;
}

// how long after its start edge a received frame is complete: the stop bit
// is sampled at 9.5 bit periods, later if the sender runs slow
inline uint32_t rx_frame_us(uint32_t bit_x16)
{
    return half_bits_us(20U, bit_x16);
}

/*
  Decode the frame whose start bit began at t0 from the edges after it.
  Source supplies them oldest first: bool available(), const Edge &peek()
  and void pop(). Edges inside the frame are consumed.

  Each bit is sampled in its middle. Like a UART, the decoder resynchronises
  on every edge: it works out which bit boundary the edge marks and times the
  following samples from it, so a sender a few percent off the baud rate does
  not drift out by the stop bit.

  Returns false, a framing error, if the stop bit is not high.
 */
template <typename Source>
bool decode_frame(uint32_t t0, Source &src, uint32_t bit_x16, uint8_t &byte)
{
    uint8_t level = 0;
    uint16_t value = 0;
    uint32_t anchor_t = t0;
    uint32_t anchor_k = 0;
    for (uint8_t bit = 1; bit <= 9; bit++) {
        uint32_t sample = anchor_t + half_bits_us(2U * (bit - anchor_k) + 1U, bit_x16);
        while (src.available() && (int32_t)(src.peek().t_us - sample) <= 0) {
            const Edge &e = src.peek();
            level = e.level;
            anchor_k = ((e.t_us - t0) * 32U + bit_x16) / (2U * bit_x16);
            if (anchor_k > bit) {
                anchor_k = bit;
            }
            anchor_t = e.t_us;
            src.pop();
            sample = anchor_t + half_bits_us(2U * (bit - anchor_k) + 1U, bit_x16);
        }
        if (level) {
            value |= (1U << (bit - 1U));
        }
    }
    byte = (uint8_t)(value & 0xFFU);
    return (value & (1U << 8)) != 0;
}

}  // namespace EdgeSerial
}  // namespace AP_HAL
