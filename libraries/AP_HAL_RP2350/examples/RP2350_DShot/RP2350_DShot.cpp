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
  DShot bench tool: throttle, commands and bidirectional eRPM.

  Driven from the console rather than from compile time settings, because a
  logic analyzer session means capturing one thing, changing one variable and
  capturing again - and reflashing between each of those loses the capture
  setup. Press '?' for the key list.

  ONE TRANSITION IS ONE WAY. Channels move from PWM onto a PIO state machine
  when the rate is chosen and the driver has no path back, so the rate is asked
  for once at startup and cannot be changed afterwards. Bidirectional mode can
  be toggled freely, as that only reloads the state machine with the other
  program.

  When reading a capture, remember that bidirectional DShot INVERTS both the
  waveform and the frame checksum. A decoder left on plain DShot will show it
  as CRC failures rather than as nothing at all.
*/

#include <AP_HAL/AP_HAL.h>

void setup();
void loop();

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#define MAX_CHANNELS 16

// how long to wait at startup for a rate to be chosen
#define RATE_PROMPT_SEC 5

#define THROTTLE_MIN 1000
#define THROTTLE_MAX 2000
#define THROTTLE_STEP 50

static uint8_t num_channels;
static uint32_t channel_mask;

static bool armed;
static bool bidir;
static bool ramping;
static uint16_t throttle = THROTTLE_MIN;
static int16_t ramp_step = THROTTLE_STEP;

static const char *mode_name = "DShot600";

static void print_help(void)
{
    hal.console->printf(
        "\n"
        "  a  arm / disarm (throttle is only sent while armed)\n"
        "  +  throttle up %u us        -  throttle down %u us\n"
        "  r  ramp the throttle up and down continuously\n"
        "  b  toggle bidirectional (eRPM) on every channel\n"
        "  1-5  send DShot beep 1-5   0  send DShot reset\n"
        "  ?  this list\n"
        "\n"
        "commands are refused while armed, so disarm before sending one\n\n",
        (unsigned)THROTTLE_STEP, (unsigned)THROTTLE_STEP);
}

/*
  Ask which DShot rate to use. This has to happen before the first
  set_output_mode(), which is why it blocks here rather than being a key in the
  main loop.
*/
static AP_HAL::RCOutput::output_mode choose_mode(void)
{
    // the prompt is reprinted each second rather than once, so that a terminal
    // attached a moment after the board booted still sees it
    for (uint8_t left = RATE_PROMPT_SEC; left > 0; left--) {
        hal.console->printf("select a rate (%u seconds left): "
                            "1 = DShot150  3 = DShot300  6 = DShot600 (default)  2 = DShot1200\n",
                            (unsigned)left);

        for (uint8_t tick = 0; tick < 100; tick++) {   // one second in 10ms steps
            while (hal.console->available() > 0) {
                switch (hal.console->read()) {
                case '1':
                    mode_name = "DShot150";
                    return AP_HAL::RCOutput::MODE_PWM_DSHOT150;
                case '3':
                    mode_name = "DShot300";
                    return AP_HAL::RCOutput::MODE_PWM_DSHOT300;
                case '6':
                    mode_name = "DShot600";
                    return AP_HAL::RCOutput::MODE_PWM_DSHOT600;
                case '2':
                    mode_name = "DShot1200";
                    return AP_HAL::RCOutput::MODE_PWM_DSHOT1200;
                default:
                    break;
                }
            }
            hal.scheduler->delay(10);
        }
    }
    return AP_HAL::RCOutput::MODE_PWM_DSHOT600;
}

static void send_command(uint8_t command, const char *what)
{
    if (armed) {
        hal.console->printf("refused: %s is not a priority command and the board is armed\n",
                            what);
        return;
    }
    hal.rcout->send_dshot_command(command, AP_HAL::RCOutput::ALL_CHANNELS);
    hal.console->printf("queued %s on every channel\n", what);
}

static void handle_key(int16_t c)
{
    switch (c) {
    case 'a':
        armed = !armed;
        hal.util->set_soft_armed(armed);
        hal.console->printf("%s\n", armed ? "armed - throttle is live" : "disarmed");
        break;
    case '+':
    case '=':
        throttle = MIN(uint16_t(throttle + THROTTLE_STEP), uint16_t(THROTTLE_MAX));
        break;
    case '-':
        throttle = MAX(int16_t(throttle - THROTTLE_STEP), int16_t(THROTTLE_MIN));
        break;
    case 'r':
        ramping = !ramping;
        hal.console->printf("ramp %s\n", ramping ? "on" : "off");
        break;
    case 'b':
        bidir = !bidir;
        hal.rcout->set_bidir_dshot_mask(bidir ? channel_mask : 0);
        hal.console->printf("bidirectional %s - a decoder must be set to %s DShot\n",
                            bidir ? "on" : "off", bidir ? "INVERTED/bidirectional" : "plain");
        break;
    case '0':
        send_command(AP_HAL::RCOutput::DSHOT_RESET, "reset");
        break;
    case '1':
        send_command(AP_HAL::RCOutput::DSHOT_BEEP1, "beep 1");
        break;
    case '2':
        send_command(AP_HAL::RCOutput::DSHOT_BEEP2, "beep 2");
        break;
    case '3':
        send_command(AP_HAL::RCOutput::DSHOT_BEEP3, "beep 3");
        break;
    case '4':
        send_command(AP_HAL::RCOutput::DSHOT_BEEP4, "beep 4");
        break;
    case '5':
        send_command(AP_HAL::RCOutput::DSHOT_BEEP5, "beep 5");
        break;
    case '?':
        print_help();
        break;
    default:
        break;
    }
}

void setup(void)
{
    hal.console->printf("\nRP2350 DShot test\n");

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
    channel_mask = (1U << num_channels) - 1;
    hal.console->printf("%u output channel(s)\n", (unsigned)num_channels);

    const AP_HAL::RCOutput::output_mode mode = choose_mode();
    hal.console->printf("using %s\n", mode_name);

    hal.rcout->set_output_mode(channel_mask, mode);
    // the pole count only affects the RPM reported to AP_ESC_Telem, not the
    // eRPM printed below; 14 is the usual count for a small motor
    hal.rcout->set_motor_poles(14);

    for (uint8_t i = 0; i < num_channels; i++) {
        hal.rcout->enable_ch(i);
    }

    print_help();
}

void loop(void)
{
    if (num_channels == 0) {
        hal.scheduler->delay(1000);
        return;
    }

    while (hal.console->available() > 0) {
        handle_key(hal.console->read());
    }

    if (ramping) {
        throttle += ramp_step;
        if (throttle >= THROTTLE_MAX) {
            throttle = THROTTLE_MAX;
            ramp_step = -THROTTLE_STEP;
        } else if (throttle <= THROTTLE_MIN) {
            throttle = THROTTLE_MIN;
            ramp_step = THROTTLE_STEP;
        }
    }

    for (uint8_t i = 0; i < num_channels; i++) {
        hal.rcout->write(i, throttle);
    }

    static uint32_t last_report_ms;
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_report_ms >= 1000) {
        last_report_ms = now_ms;
        hal.console->printf("%s %s throttle %u", mode_name,
                            armed ? "armed" : "disarmed", (unsigned)throttle);
        if (bidir) {
            /*
              With no ESC attached nothing ever answers, so every channel
              reports 0 eRPM at a 100% error rate. That is the expected result
              rather than a failure - what it proves is that the driver keeps
              re-arming the state machine and sending frames anyway.
            */
            for (uint8_t i = 0; i < num_channels; i++) {
                hal.console->printf("  ch%u %u erpm/%.0f%%", (unsigned)i,
                                    (unsigned)hal.rcout->get_erpm(i),
                                    (double)hal.rcout->get_erpm_error_rate(i));
            }
        }
        hal.console->printf("\n");
    }

    hal.scheduler->delay(20);
}

AP_HAL_MAIN();
