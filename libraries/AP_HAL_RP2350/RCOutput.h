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

#include <hardware/pio.h>

#ifndef AP_HAL_RCOUTPUT_ENABLED
#define AP_HAL_RCOUTPUT_ENABLED defined(HAL_RP2350_RCOUT_CHANNELS)
#endif

#if AP_HAL_RCOUTPUT_ENABLED

// the PWM counters are run at 1MHz so that a level is a pulse width in
// microseconds, which is the unit AP_HAL::RCOutput works in
#define RP2350_PWM_COUNT_HZ 1000000

#ifndef RP2350_RCOUT_DEFAULT_FREQ
#define RP2350_RCOUT_DEFAULT_FREQ 50
#endif

namespace RP2350
{

class RCOutput : public AP_HAL::RCOutput
{
public:
    void init() override;

    void set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t chan) override;

    void enable_ch(uint8_t chan) override;
    void disable_ch(uint8_t chan) override;

    void write(uint8_t chan, uint16_t period_us) override;
    uint16_t read(uint8_t chan) override;
    void read(uint16_t *period_us, uint8_t len) override;

    void cork() override;
    void push() override;

    void set_output_mode(uint32_t mask, enum output_mode mode) override;

private:
    // move one channel from its PWM slice to a PIO state machine running the
    // DShot program at the given wire bit rate
    bool dshot_configure(uint8_t chan, uint32_t bitrate);

    // build a DShot frame: 11 bit value, telemetry request, 4 bit checksum
    static uint16_t dshot_packet(uint16_t value, bool telem_request);

    // push a frame to every DShot channel; runs from the timer thread, as
    // ESCs disarm if frames stop arriving
    void dshot_send();

    // push one channel's pulse width to the hardware
    void apply(uint8_t chan);

    static uint8_t num_channels();

    // the GPIO each output channel drives, from hwdef.dat
    static const uint8_t channel_gpio[];

    uint16_t _period_us[16];
    uint16_t _pending_us[16];
    uint32_t _enabled_mask;

    // set while cork() is holding writes back for a synchronised push()
    bool _corked;
    uint32_t _corked_mask;

    // one entry per PWM slice; both channels of a slice share the counter,
    // so they cannot be given different frequencies
    uint16_t _slice_freq_hz[12];

    // channels moved from PWM to DShot
    uint32_t _dshot_mask;
    struct {
        PIO pio;
        uint8_t sm;
    } _dshot[16];

    // where the DShot program sits in each PIO block's instruction memory,
    // or -1 if it has not been loaded into that block yet
    int8_t _pio_offset[NUM_PIOS];
};

}

#endif // AP_HAL_RCOUTPUT_ENABLED
