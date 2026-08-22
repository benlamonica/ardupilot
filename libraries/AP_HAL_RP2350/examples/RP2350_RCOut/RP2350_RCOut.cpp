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
  PWM output test for a logic analyzer.

  The shared examples/RCOutput ramps every channel through the same widths at
  once, which cannot tell you whether channel 3 is really driving GPIO 9. This
  gives every channel a different, constant pulse width instead, so one capture
  shows at a glance that each output is the one it claims to be.

  It also drives alternate slices at different frame rates, because that is the
  constraint most likely to catch someone out on this chip: a GPIO's PWM slice
  is (gpio >> 1) & 7, so outputs pair up and BOTH CHANNELS OF A PAIR SHARE ONE
  COUNTER. A pair can hold different widths but not different rates. The report
  printed at startup shows what each channel actually ended up at.
*/

#include <AP_HAL/AP_HAL.h>

void setup();
void loop();

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#define MAX_CHANNELS 16

// two rates far enough apart to tell apart on a capture without measuring
#define SLOW_HZ 50
#define FAST_HZ 400

static uint8_t num_channels;

// each channel gets a width this far above the last, spanning the usual
// 1000-2000us servo range
static uint16_t width_for(uint8_t chan)
{
    return 1000 + chan * 125;
}

void setup(void)
{
    hal.console->printf("\nRP2350 RCOutput test\n");

    /*
      get_freq() returns 0 for a channel the board does not declare, and
      init() has already set every declared one to the default rate, so this
      counts the outputs through the AP_HAL interface rather than by including
      a driver header.
    */
    for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
        if (hal.rcout->get_freq(i) == 0) {
            break;
        }
        num_channels = i + 1;
    }

    if (num_channels == 0) {
        hal.console->printf("no outputs declared - check RP2350_RCOUT in hwdef.dat\n");
        return;
    }
    hal.console->printf("%u output channel(s)\n", (unsigned)num_channels);

    // alternate the rate every two channels, which is one PWM slice
    for (uint8_t i = 0; i < num_channels; i++) {
        const uint16_t hz = ((i / 2) & 1) ? FAST_HZ : SLOW_HZ;
        hal.rcout->set_freq(1U << i, hz);
    }

    for (uint8_t i = 0; i < num_channels; i++) {
        hal.rcout->enable_ch(i);
        hal.rcout->write(i, width_for(i));
    }

    hal.console->printf("chan  requested  width_us  freq_hz\n");
    for (uint8_t i = 0; i < num_channels; i++) {
        const uint16_t hz = ((i / 2) & 1) ? FAST_HZ : SLOW_HZ;
        const uint16_t got = hal.rcout->get_freq(i);
        hal.console->printf("%4u  %9u  %8u  %7u%s\n", (unsigned)i, (unsigned)hz,
                            (unsigned)hal.rcout->read(i), (unsigned)got,
                            got == hz ? "" : "   <- changed by its slice partner");
    }

    hal.console->printf("\nexpect %u distinct widths, and one shared period per pair\n",
                        (unsigned)num_channels);
}

void loop(void)
{
    if (num_channels == 0) {
        hal.scheduler->delay(1000);
        return;
    }

    /*
      Rewrite the same values rather than sitting idle, so that a capture taken
      at any time is of a live output rather than of one that merely has not
      been touched since boot.
    */
    hal.rcout->cork();
    for (uint8_t i = 0; i < num_channels; i++) {
        hal.rcout->write(i, width_for(i));
    }
    hal.rcout->push();

    hal.scheduler->delay(20);
}

AP_HAL_MAIN();
