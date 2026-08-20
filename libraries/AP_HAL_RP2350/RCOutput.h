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

#include <AP_HAL/utility/RingBuffer.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>

#include <hardware/pio.h>

#ifndef AP_HAL_RCOUTPUT_ENABLED
#define AP_HAL_RCOUTPUT_ENABLED defined(HAL_RP2350_RCOUT_CHANNELS)
#endif

#if AP_HAL_RCOUTPUT_ENABLED

// the PWM counters are run at 1MHz so that a level is a pulse width in
// microseconds, which is the unit AP_HAL::RCOutput works in
#define RP2350_PWM_COUNT_HZ 1000000

// DShot frames go out from the 1kHz timer process, so this is how long a
// command occupies per repeat
#define RP2350_DSHOT_PERIOD_US 1000

// how long the counters behind get_erpm_error_rate() run before restarting,
// so that the rate reported stays a recent one
#define RP2350_ERPM_STATS_MS 5000

#ifndef RP2350_RCOUT_DEFAULT_FREQ
#define RP2350_RCOUT_DEFAULT_FREQ 50
#endif

namespace RP2350
{

class RCOutput : public AP_HAL::RCOutput, AP_ESC_Telem_Backend
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

    void send_dshot_command(uint8_t command, uint8_t chan = ALL_CHANNELS,
                            uint32_t command_timeout_ms = 0, uint16_t repeat_count = 10,
                            bool priority = false) override;

    uint32_t get_dshot_period_us() const override { return RP2350_DSHOT_PERIOD_US; }

    void set_bidir_dshot_mask(uint32_t mask) override;
    void set_motor_poles(uint8_t poles) override { _motor_poles = poles; }

    uint16_t get_erpm(uint8_t chan) const override {
        return chan < ARRAY_SIZE(_bdshot.erpm) ? _bdshot.erpm[chan] : 0;
    }
    float get_erpm_error_rate(uint8_t chan) const override {
        if (chan >= ARRAY_SIZE(_bdshot.errors)) {
            return 100.0f;
        }
        return 100.0f * float(_bdshot.errors[chan]) /
               (1 + _bdshot.errors[chan] + _bdshot.clean[chan]);
    }
    bool new_erpm() override { return _bdshot.update_mask != 0; }
    uint32_t read_erpm(uint16_t *erpm, uint8_t len) override;

private:
    // an eRPM response that did not decode
    static const uint16_t INVALID_ERPM = 0xffffU;
    // the value an ESC sends for a motor that is not turning
    static const uint16_t ZERO_ERPM = 0x0fffU;

    // where a DShot program sits in one PIO block's instruction memory,
    // loading it there if this is the first channel to need it; -1 if that
    // block's instruction memory cannot hold it
    int8_t pio_program_offset(uint8_t block, bool bidir);

    // move one channel from its PWM slice to a PIO state machine running the
    // DShot program at the given wire bit rate, or reload one already moved
    bool dshot_configure(uint8_t chan, uint32_t bitrate);

    // build a DShot frame: 11 bit value, telemetry request, 4 bit checksum
    static uint16_t dshot_packet(uint16_t value, bool telem_request, bool bidir);

    // take the eRPM the ESC sent in answer to the previous frame, and put a
    // state machine still waiting for one back at the top of its program
    void bdshot_receive(uint8_t chan);

    // turn one sampled 21 bit response into the telemetry value it carries,
    // or INVALID_ERPM
    static uint16_t bdshot_decode(uint32_t raw);

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

    /*
      A DShot command occupies the throttle field, so it displaces the
      throttle for as many frames as `cycles` says. Commands are queued
      because callers send them in sequences, such as a run of beeps.
    */
    struct DshotCommandPacket {
        uint8_t command;
        uint8_t chan;      // an output channel, or ALL_CHANNELS
        uint32_t cycles;
    };
    ObjectBuffer<DshotCommandPacket> _dshot_command_queue{8};
    DshotCommandPacket _dshot_command;

    // channels moved from PWM to DShot, and the wire bit rate they were
    // moved at, which a later change of direction has to be reapplied at
    uint32_t _dshot_mask;
    uint32_t _dshot_bitrate;
    struct {
        PIO pio;
        uint8_t sm;
        uint8_t entry;  // program entry point, to restart a stalled machine
    } _dshot[16];

    // where each DShot program sits in each PIO block's instruction memory,
    // or -1 if it has not been loaded into that block yet. Both can be
    // resident at once, so a board may mix bidirectional and plain channels.
    int8_t _pio_offset[NUM_PIOS];
    int8_t _pio_bdshot_offset[NUM_PIOS];

    // channels the ESC answers on with eRPM, from SERVO_BLH_BDMASK
    uint32_t _bidir_mask;
    uint8_t _motor_poles = 14;

    struct {
        uint16_t erpm[16];
        uint32_t update_mask;
        uint16_t errors[16];
        uint16_t clean[16];
        uint32_t stats_ms[16];
    } _bdshot;
};

}

#endif // AP_HAL_RCOUTPUT_ENABLED
