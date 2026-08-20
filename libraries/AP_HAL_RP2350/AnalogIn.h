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

#pragma once

#include <AP_HAL/AP_HAL.h>
#include "HAL_RP2350_Namespace.h"
#include "Semaphores.h"

#ifndef AP_HAL_ANALOGIN_ENABLED
#define AP_HAL_ANALOGIN_ENABLED defined(HAL_RP2350_ADC_PINS)
#endif

#if AP_HAL_ANALOGIN_ENABLED

/*
  the ADC measures against its own reference pin rather than the 5V rail, so
  this is both the full scale voltage and what a ratiometric sensor on this
  board is referenced to
*/
#ifndef HAL_RP2350_ADC_VREF
#define HAL_RP2350_ADC_VREF 3.3f
#endif

// how many AnalogSource objects can be handed out at once
#define RP2350_ANALOG_MAX_CHANNELS 8

namespace RP2350
{

class AnalogSource : public AP_HAL::AnalogSource
{
public:
    friend class AnalogIn;

    AnalogSource(int16_t ardupin, uint8_t adc_channel, float scaler);

    float read_average() override;
    float read_latest() override;
    bool set_pin(uint8_t p) override WARN_IF_UNUSED;
    float voltage_average() override;
    float voltage_latest() override;
    float voltage_average_ratiometric() override;

private:
    // take one sample; only ever called from the timer thread
    void _add_value();

    // the pin number as it appears in ArduPilot parameters
    int16_t _ardupin;
    // hardware ADC input, as passed to adc_select_input()
    uint8_t _adc_channel;
    // external divider ratio, from hwdef.dat
    float _scaler;

    float _value;
    float _latest_value;
    uint16_t _sum_count;
    float _sum_value;

    Semaphore _semaphore;
};

class AnalogIn : public AP_HAL::AnalogIn
{
public:
    friend class AnalogSource;

    void init() override;
    AP_HAL::AnalogSource* channel(int16_t pin) override;
    bool valid_analog_pin(uint16_t pin) const override;

    float board_voltage(void) override { return HAL_RP2350_ADC_VREF; }

    // index into pin_config for an ArduPilot pin number, or -1
    static int8_t find_pinconfig(int16_t ardupin);

    struct pin_info {
        uint8_t channel;   // hardware ADC input
        float scaling;     // external divider ratio
        uint8_t ardupin;   // pin number as used in ArduPilot parameters
    };

    static const pin_info pin_config[];

private:
    void _timer_tick(void);

    AnalogSource *_channels[RP2350_ANALOG_MAX_CHANNELS];
};

}

#endif // AP_HAL_ANALOGIN_ENABLED
