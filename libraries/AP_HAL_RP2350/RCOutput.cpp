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
#include <hardware/pwm.h>

using namespace RP2350;

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

    // set_freq() writes the wrap register, which is per slice
    set_freq((1U << n) - 1, RP2350_RCOUT_DEFAULT_FREQ);
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
