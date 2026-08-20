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

#include "RCOutput.h"

#if AP_HAL_RCOUTPUT_ENABLED

#include <AP_Math/AP_Math.h>

#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/pwm.h>

#include "dshot.pio.h"

using namespace RP2350;

extern const AP_HAL::HAL &hal;

const uint8_t RCOutput::channel_gpio[] = { HAL_RP2350_RCOUT_CHANNELS };

uint8_t RCOutput::num_channels()
{
    return MIN(ARRAY_SIZE(channel_gpio), 16U);
}

void RCOutput::init()
{
    const uint8_t n = num_channels();

    for (uint8_t i = 0; i < n; i++) {
        const uint8_t gpio = channel_gpio[i];
        const uint slice = pwm_gpio_to_slice_num(gpio);

        gpio_set_function(gpio, GPIO_FUNC_PWM);

        /*
          Divide the system clock down to 1MHz so that a channel level is
          simply the pulse width in microseconds. At that rate a 16 bit wrap
          reaches 65535us, so any frame rate down to about 16Hz is reachable.
        */
        pwm_set_clkdiv(slice, clock_get_hz(clk_sys) / (float)RP2350_PWM_COUNT_HZ);

        // start disarmed: a level of zero holds the output low
        pwm_set_gpio_level(gpio, 0);
        pwm_set_enabled(slice, true);
    }

    for (uint8_t b = 0; b < NUM_PIOS; b++) {
        _pio_offset[b] = -1;
    }

    // set_freq() writes the wrap register, which is per slice
    set_freq((1U << n) - 1, RP2350_RCOUT_DEFAULT_FREQ);

    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&RCOutput::dshot_send, void));
}

/*
  DShot is a serial protocol rather than a pulse width, so it cannot come from
  a PWM slice: each frame is 16 bits and every bit is itself a short pulse.
  A PIO state machine generates that encoding, which also means any GPIO can
  carry DShot, and that the paired-slice frequency restriction does not apply
  to a channel once it has been moved here.
*/
void RCOutput::set_output_mode(uint32_t mask, enum output_mode mode)
{
    uint32_t bitrate;
    switch (mode) {
    case MODE_PWM_DSHOT150:
        bitrate = 150000;
        break;
    case MODE_PWM_DSHOT300:
        bitrate = 300000;
        break;
    case MODE_PWM_DSHOT600:
        bitrate = 600000;
        break;
    case MODE_PWM_DSHOT1200:
        bitrate = 1200000;
        break;
    default:
        // everything else is left on the PWM slices, which is the default
        return;
    }

    for (uint8_t i = 0; i < num_channels(); i++) {
        if ((mask & (1U << i)) == 0 || (_dshot_mask & (1U << i)) != 0) {
            continue;
        }
        if (dshot_configure(i, bitrate)) {
            _dshot_mask |= (1U << i);
        }
    }
}

bool RCOutput::dshot_configure(uint8_t chan, uint32_t bitrate)
{
    const uint8_t gpio = channel_gpio[chan];

    for (uint8_t b = 0; b < NUM_PIOS; b++) {
        PIO pio = pio_get_instance(b);

        const int sm = pio_claim_unused_sm(pio, false);
        if (sm < 0) {
            // no free state machine in this block, try the next one
            continue;
        }
        if (_pio_offset[b] < 0) {
            // first DShot channel on this block, so load the program; all
            // four of its state machines then share the one copy
            if (!pio_can_add_program(pio, &dshot_program)) {
                pio_sm_unclaim(pio, sm);
                continue;
            }
            _pio_offset[b] = pio_add_program(pio, &dshot_program);
        }

        pio_sm_config c = dshot_program_get_default_config(_pio_offset[b]);
        sm_config_set_sideset_pins(&c, gpio);
        /*
          DShot sends most significant bit first, so shift left, and autopull
          at 16 bits so that the state machine stalls between frames with the
          line held low rather than running the frame back to back.
        */
        sm_config_set_out_shift(&c, false, true, 16);
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
        // the program spends T1+T2+T3 = 8 cycles on each bit
        sm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / (bitrate * 8));

        pio_gpio_init(pio, gpio);
        pio_sm_set_consecutive_pindirs(pio, sm, gpio, 1, true);
        pio_sm_init(pio, sm, _pio_offset[b], &c);
        pio_sm_set_enabled(pio, sm, true);

        _dshot[chan].pio = pio;
        _dshot[chan].sm = sm;
        return true;
    }
    return false;
}

uint16_t RCOutput::dshot_packet(uint16_t value, bool telem_request)
{
    uint16_t packet = (value << 1) | (telem_request ? 1 : 0);

    // the checksum is the exclusive or of the three nibbles above it
    uint16_t csum = 0;
    uint16_t csum_data = packet;
    for (uint8_t i = 0; i < 3; i++) {
        csum ^= csum_data;
        csum_data >>= 4;
    }

    return (packet << 4) | (csum & 0xf);
}

void RCOutput::dshot_send()
{
    if (_dshot_mask == 0) {
        return;
    }
    const bool armed = hal.util->get_soft_armed();

    for (uint8_t i = 0; i < num_channels(); i++) {
        if ((_dshot_mask & (1U << i)) == 0) {
            continue;
        }

        uint16_t value = 0;
        const uint16_t pwm = _period_us[i];
        if (armed && pwm != 0 && (_enabled_mask & (1U << i)) != 0) {
            // the same mapping the other HALs use: 1000-2000us becomes the
            // 48-2047 throttle range, with zero reserved for "stopped"
            const uint16_t p = constrain_int16(pwm, 1000, 2000);
            value = MIN(2 * (p - 1000), 1999);
            if (value != 0) {
                value += DSHOT_ZERO_THROTTLE;
            }
        }

        const uint32_t frame = dshot_packet(value, false);
        if (!pio_sm_is_tx_fifo_full(_dshot[i].pio, _dshot[i].sm)) {
            // left justified, because the state machine shifts out of the top
            // of the output shift register
            pio_sm_put(_dshot[i].pio, _dshot[i].sm, frame << 16);
        }
    }
}

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    if (freq_hz == 0) {
        return;
    }
    const uint32_t wrap = (RP2350_PWM_COUNT_HZ / freq_hz) - 1;
    if (wrap > UINT16_MAX) {
        // slower than the counter can reach without changing the divisor
        return;
    }

    for (uint8_t i = 0; i < num_channels(); i++) {
        if ((chmask & (1U << i)) == 0) {
            continue;
        }
        const uint slice = pwm_gpio_to_slice_num(channel_gpio[i]);
        pwm_set_wrap(slice, (uint16_t)wrap);
        /*
          Both channels of a slice share this counter, so a channel that was
          not in chmask but shares a slice with one that was has just had its
          rate changed too. Recording the rate per slice rather than per
          channel is what makes get_freq() report that honestly.
        */
        _slice_freq_hz[slice] = freq_hz;
    }
}

uint16_t RCOutput::get_freq(uint8_t chan)
{
    if (chan >= num_channels()) {
        return 0;
    }
    return _slice_freq_hz[pwm_gpio_to_slice_num(channel_gpio[chan])];
}

void RCOutput::enable_ch(uint8_t chan)
{
    if (chan >= num_channels()) {
        return;
    }
    _enabled_mask |= (1U << chan);
    apply(chan);
}

void RCOutput::disable_ch(uint8_t chan)
{
    if (chan >= num_channels()) {
        return;
    }
    _enabled_mask &= ~(1U << chan);
    // a level of zero holds the output low without disturbing the slice, and
    // so without disturbing the other channel sharing it
    pwm_set_gpio_level(channel_gpio[chan], 0);
}

void RCOutput::write(uint8_t chan, uint16_t period_us)
{
    if (chan >= num_channels()) {
        return;
    }
    if (_corked) {
        _pending_us[chan] = period_us;
        _corked_mask |= (1U << chan);
        return;
    }
    _period_us[chan] = period_us;
    apply(chan);
}

void RCOutput::apply(uint8_t chan)
{
    if ((_enabled_mask & (1U << chan)) == 0) {
        return;
    }
    if ((_dshot_mask & (1U << chan)) != 0) {
        // the GPIO belongs to a PIO state machine now; dshot_send() picks the
        // value up from _period_us on its next pass
        return;
    }
    pwm_set_gpio_level(channel_gpio[chan], _period_us[chan]);
}

uint16_t RCOutput::read(uint8_t chan)
{
    if (chan >= num_channels()) {
        return 0;
    }
    return _period_us[chan];
}

void RCOutput::read(uint16_t *period_us, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        period_us[i] = read(i);
    }
}

void RCOutput::cork()
{
    _corked = true;
}

void RCOutput::push()
{
    if (!_corked) {
        return;
    }
    _corked = false;
    for (uint8_t i = 0; i < num_channels(); i++) {
        if (_corked_mask & (1U << i)) {
            _period_us[i] = _pending_us[i];
            apply(i);
        }
    }
    _corked_mask = 0;
}

#endif // AP_HAL_RCOUTPUT_ENABLED
