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

#include "AnalogIn.h"

#if AP_HAL_ANALOGIN_ENABLED

#include <AP_Math/AP_Math.h>

#include <hardware/adc.h>

using namespace RP2350;

extern const AP_HAL::HAL &hal;

/*
  scaling table between ADC channel and ArduPilot pin number, with the
  external divider ratio on each, from the board's RP2350_ADC_PIN lines
*/
const AnalogIn::pin_info AnalogIn::pin_config[] = { HAL_RP2350_ADC_PINS };

#define RP2350_ADC_NUM_PINS ARRAY_SIZE(AnalogIn::pin_config)

/*
  the SAR is 12 bits, and the SDK's own examples spread the reference over
  1<<12 counts rather than over the 4095 count full scale reading
*/
#define RP2350_ADC_COUNT_TO_VOLTS (HAL_RP2350_ADC_VREF / 4096.0f)

AnalogSource::AnalogSource(int16_t ardupin, uint8_t adc_channel, float scaler) :
    _ardupin(ardupin),
    _adc_channel(adc_channel),
    _scaler(scaler),
    _value(0),
    _latest_value(0),
    _sum_count(0),
    _sum_value(0)
{}

/*
  take one sample. This runs in the timer thread, which is the only reader of
  the ADC: keeping it that way is what makes selecting the input and reading
  it safe without locking the hardware.
*/
void AnalogSource::_add_value()
{
    if (_ardupin == ANALOG_INPUT_NONE) {
        return;
    }

    adc_select_input(_adc_channel);
    const float value = adc_read();

    WITH_SEMAPHORE(_semaphore);

    _latest_value = value;
    _sum_value += value;
    _sum_count++;

    if (_sum_count == 254) {
        // keep the average rolling rather than letting it saturate
        _sum_value /= 2;
        _sum_count /= 2;
    }
}

float AnalogSource::read_average()
{
    if (_ardupin == ANALOG_INPUT_NONE) {
        return 0;
    }

    WITH_SEMAPHORE(_semaphore);

    if (_sum_count == 0) {
        // nothing new since the last call; the timer thread owns the ADC, so
        // report the last average rather than reading the hardware here
        return _value;
    }

    _value = _sum_value / _sum_count;
    _sum_value = 0;
    _sum_count = 0;

    return _value;
}

float AnalogSource::read_latest()
{
    WITH_SEMAPHORE(_semaphore);
    return _latest_value;
}

float AnalogSource::voltage_average()
{
    return read_average() * RP2350_ADC_COUNT_TO_VOLTS * _scaler;
}

float AnalogSource::voltage_latest()
{
    return read_latest() * RP2350_ADC_COUNT_TO_VOLTS * _scaler;
}

float AnalogSource::voltage_average_ratiometric()
{
    // the ADC reference is the same rail a ratiometric sensor would be run
    // from on this board, so this is the plain average
    return voltage_average();
}

bool AnalogSource::set_pin(uint8_t pin)
{
    if (_ardupin == pin) {
        return true;
    }

    const int8_t idx = AnalogIn::find_pinconfig(pin);
    if (pin != ANALOG_INPUT_NONE && idx == -1) {
        // not a pin this board brings out to the ADC
        return false;
    }

    WITH_SEMAPHORE(_semaphore);

    _ardupin = pin;
    if (idx == -1) {
        _adc_channel = 0;
        _scaler = 0;
    } else {
        _adc_channel = AnalogIn::pin_config[idx].channel;
        _scaler = AnalogIn::pin_config[idx].scaling;
    }

    // the accumulated samples belong to the old pin
    _value = 0;
    _latest_value = 0;
    _sum_value = 0;
    _sum_count = 0;

    return true;
}

void AnalogIn::init()
{
    adc_init();

    for (uint8_t i = 0; i < RP2350_ADC_NUM_PINS; i++) {
        const uint8_t chan = pin_config[i].channel;
        if (chan == ADC_TEMPERATURE_CHANNEL_NUM) {
            adc_set_temp_sensor_enabled(true);
        } else {
            // the analog inputs are the last GPIOs on the part, in channel order
            adc_gpio_init(ADC_BASE_PIN + chan);
        }
    }

    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AnalogIn::_timer_tick, void));
}

void AnalogIn::_timer_tick(void)
{
    for (uint8_t i = 0; i < RP2350_ANALOG_MAX_CHANNELS; i++) {
        if (_channels[i] != nullptr) {
            _channels[i]->_add_value();
        }
    }
}

int8_t AnalogIn::find_pinconfig(int16_t ardupin)
{
    for (uint8_t i = 0; i < RP2350_ADC_NUM_PINS; i++) {
        if (pin_config[i].ardupin == ardupin) {
            return i;
        }
    }
    return -1;
}

bool AnalogIn::valid_analog_pin(uint16_t pin) const
{
    return find_pinconfig(pin) != -1;
}

AP_HAL::AnalogSource *AnalogIn::channel(int16_t ardupin)
{
    if (ardupin < 0) {
        ardupin = ANALOG_INPUT_NONE;
    }

    const int8_t idx = find_pinconfig(ardupin);
    if (ardupin != ANALOG_INPUT_NONE && idx == -1) {
        // let it through unattached so that set_pin() can still move it to a
        // real pin later
        ardupin = ANALOG_INPUT_NONE;
    }

    const uint8_t chan = (idx == -1) ? 0 : pin_config[idx].channel;
    const float scaler = (idx == -1) ? 0 : pin_config[idx].scaling;

    for (uint8_t i = 0; i < RP2350_ANALOG_MAX_CHANNELS; i++) {
        if (_channels[i] == nullptr) {
            _channels[i] = NEW_NOTHROW AnalogSource(ardupin, chan, scaler);
            return _channels[i];
        }
    }
    return nullptr;
}

#endif // AP_HAL_ANALOGIN_ENABLED
