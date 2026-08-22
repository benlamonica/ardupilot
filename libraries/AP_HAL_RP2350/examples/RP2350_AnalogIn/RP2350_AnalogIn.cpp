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

/*
  Read every analog pin this board declares.

  The shared examples/AnalogIn walks pins 0-15, which are not the pin numbers
  an RP2350 board uses - the generic board numbers its ADC pins 26-28 so that
  they match the silkscreen. Rather than hard coding that, this asks the driver
  which pins exist, so it keeps working for any board's hwdef.dat.

  Jumper a pin to 3V3 and it should read the reference voltage; jumper it to
  AGND and it should read close to zero. A floating pin reads somewhere in
  between and drifts, which is not a fault.
*/

#include <AP_HAL/AP_HAL.h>

void setup();
void loop();

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

// the driver hands out a limited number of sources; this is the most any
// RP2350 package brings out, and channel() returning null is handled anyway
#define MAX_PINS 8

// ArduPilot pin numbers are the GPIO numbers on this HAL, and the RP2350B
// brings out 47 of them
#define MAX_ARDUPIN 47

static AP_HAL::AnalogSource *sources[MAX_PINS];
static uint8_t pins[MAX_PINS];
static uint8_t num_pins;

void setup(void)
{
    hal.console->printf("\nRP2350 AnalogIn test\n");
    hal.console->printf("analog reference %.3fV\n", (double)hal.analogin->board_voltage());

    /*
      valid_analog_pin() answers from the table hwdef.dat generated, so this
      discovers the board's pins through the AP_HAL interface rather than by
      including a driver header.
    */
    for (uint16_t p = 0; p <= MAX_ARDUPIN && num_pins < MAX_PINS; p++) {
        if (!hal.analogin->valid_analog_pin(p)) {
            continue;
        }
        AP_HAL::AnalogSource *src = hal.analogin->channel(p);
        if (src == nullptr) {
            hal.console->printf("pin %u declared but no source available\n", (unsigned)p);
            continue;
        }
        sources[num_pins] = src;
        pins[num_pins] = p;
        num_pins++;
    }

    if (num_pins == 0) {
        hal.console->printf("no analog pins declared - check RP2350_ADC_PIN in hwdef.dat\n");
        return;
    }

    hal.console->printf("%u pin(s):", (unsigned)num_pins);
    for (uint8_t i = 0; i < num_pins; i++) {
        hal.console->printf(" %u", (unsigned)pins[i]);
    }
    hal.console->printf("\n");
}

void loop(void)
{
    if (num_pins == 0) {
        hal.scheduler->delay(1000);
        return;
    }

    for (uint8_t i = 0; i < num_pins; i++) {
        // the average is what a battery monitor would use; the latest sample
        // is shown beside it so that a noisy or floating pin is obvious
        hal.console->printf("pin %2u %6.3fV (latest %6.3fV)  ", (unsigned)pins[i],
                            (double)sources[i]->voltage_average(),
                            (double)sources[i]->voltage_latest());
    }
    hal.console->printf("\n");

    hal.scheduler->delay(500);
}

AP_HAL_MAIN();
