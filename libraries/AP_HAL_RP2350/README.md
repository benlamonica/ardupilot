# ArduPilot port to the RP2350 (Raspberry Pi Pico 2)

This HAL wraps the Raspberry Pi Pico-SDK, in the same way `AP_HAL_ESP32` wraps
Espressif's ESP-IDF: the vendor SDK supplies the low-level peripheral drivers and
this directory adapts them to the `AP_HAL` interfaces.

**Status: early bring-up.** The build produces a complete firmware image, and runs
under FreeRTOS on both Cortex-M33 cores. The Scheduler, Semaphores, GPIO, serial
ports, parameter storage, analog inputs, PWM outputs and Util are implemented; SPI,
I2C and RC input are still wired to `AP_HAL_Empty` stubs, so there are no sensors and
no way to fly it. It has not been flight tested, and it is not airworthy.

## Serial ports

`SERIAL0` is the USB CDC console. The RP2350's two hardware UARTs become `SERIAL1`
and `SERIAL2`, in the order the board's `hwdef.dat` declares them:

```
# RP2350_SERIAL <uart> <RX GPIO> <TX GPIO>
RP2350_SERIAL uart0 1 0
```

The generator checks each GPIO against the RP2350's bank 0 function table, so a pin
that cannot carry that signal is a configure-time error rather than a silent
failure on the bench.

Receive is interrupt driven, because at the higher GPS baud rates the 32-byte
hardware FIFO overflows well inside the 1 kHz thread tick. Transmit is polled from
that tick instead: the PL011 raises its TX interrupt only as the FIFO drains past
the threshold, never from an empty FIFO, so an interrupt-driven path would still
have to be primed by the writing thread and the queue would end up with two
consumers. The cost is that a *sustained* stream is capped at one FIFO per tick,
around 320 kbaud; bursts go out at full line rate. DMA would lift that if a port
ever needs it.

## Parameter storage

Parameters live in two 64k erase blocks at the very top of the QSPI flash, above
the firmware, driven by the shared `AP_FlashStorage` log. A UF2 or picotool load
only rewrites the sectors the image itself covers, so parameters survive a firmware
update; `_flash_load()` panics if a growing image ever reaches the storage region.

Two things are specific to this chip. The bootrom routine behind
`flash_range_program()` only takes whole 256 byte pages, so `AP_FlashStorage` uses a
256 byte block layout here (`AP_FLASHSTORAGE_TYPE_RP2350`, alongside the existing H7
and G4 chunk-write types). That makes `HAL_STORAGE_SIZE` 65*254 rather than a round
16384: it has to be a whole number of blocks, because `load_sector()` rejects a block
reaching past the end of storage, so a partial trailing block can be written but
never read back.

Erasing or programming stops *both* cores, since the other one would otherwise fault
fetching from XIP, so every access goes through `pico_flash`'s `flash_safe_execute()`.
Erases are gated on being disarmed. The cost of the coarse blocks is that the log
fills after about 190 line writes and then needs a sector switch and a block erase;
raising `STORAGE_SECTOR_SIZE` trades flash space for fewer erases.

`AP_FlashStorage` itself is exercised by `examples/FlashTest`, which is worth running
for anything touching the storage arithmetic: it writes random offsets across the
whole buffer, where `StorageTest` only reaches the areas StorageManager declares, and
that gap has already hidden a real bug once.

Run it on the board rather than on SITL. It simulates the flash in RAM, so it tests
the layout rather than the hardware either way, but a standalone SITL example cannot
show `hal.console` output at all: the drain happens in `_timer_tick()`, which is only
reached from `stop_clock()` when a SITL vehicle state object exists. On SITL a
failure is still visible, because panics go through stdio, but a pass is not.

```sh
./waf --targets examples/FlashTest   # prints TEST PASSED on the console
```

## Analog inputs

The 12 bit ADC is declared per board in `hwdef.dat`:

```
# RP2350_ADC_PIN <ADC input> <divider ratio> <ArduPilot pin number>
RP2350_ADC_PIN 0 1.0 26
```

The ArduPilot pin number is what a parameter such as `BATT_VOLT_PIN` refers to;
the generic board uses the GPIO number so that it matches the silkscreen. The
divider ratio is the external scaling, so a battery sense divider goes here rather
than in the driver.

Readings are averaged by a timer process registered in `AnalogIn::init()`, which is
the only reader of the ADC: keeping the hardware to one thread is what makes
selecting an input and reading it safe without locking. `board_voltage()` returns the
analog reference rather than a 5V rail, because that is what the ADC measures against
and what a ratiometric sensor on this board would be powered from.

Measured against the Pico's own rails, an input tied to 3V3 reads 3.297V and one tied
to AGND reads 0.001V.

## PWM outputs

Motor and servo outputs are declared per board in `hwdef.dat`, in output channel
order:

```
# RP2350_RCOUT <GPIO>
RP2350_RCOUT 6
RP2350_RCOUT 7
```

The counters are divided down to 1MHz so that a channel level is the pulse width in
microseconds directly, which is the unit `AP_HAL::RCOutput` works in. A 16 bit wrap
at that rate reaches 65535us, so any frame rate down to about 16Hz is available
without touching the divisor.

The constraint to know about is that a GPIO's PWM slice is `(gpio >> 1) & 7` and its
channel within that slice is `gpio & 1`, so GPIOs pair up, and **both channels of a
slice share one counter**. A pair can hold different pulse widths but not different
frame rates: `set_freq()` on one channel changes its partner too. `get_freq()`
reports the rate per slice rather than per channel so that it stays honest about
this. Keep outputs that need different rates on different slices.

Outputs sit low until a channel is both enabled and written, so nothing twitches
during boot.

Verified with a logic analyzer on GP6-GP13, driving eight distinct widths across the
1000-2000us range at a mix of 50Hz and 400Hz: measured widths and periods matched the
requested values, and each slice pair shared a period as expected.

### DShot

DShot is a serial protocol rather than a pulse width - each frame is 16 bits and every
bit is itself a short pulse - so it cannot come from a PWM slice. `set_output_mode()`
instead moves a channel to a PIO state machine running `dshot.pio`, which generates
the bit encoding in hardware. Two useful consequences: any GPIO can carry DShot, since
PIO is not tied to particular pins the way a DMA-capable timer is on STM32; and the
paired-slice frequency restriction above stops applying to a channel once it moves.

Each bit is split into T1 + T2 + T3 = 8 state machine cycles, leaving a '0' high for
3/8 of the bit and a '1' for 6/8, so the state machine simply runs at eight times the
wire bit rate. Because the state machine does the encoding, the CPU only writes one
word per motor per frame, and no DMA is needed. Frames are sent from a timer process
rather than from `push()`, as ESCs disarm if frames stop arriving.

`dshot.pio.h` is generated from `dshot.pio` and checked in beside it, because
ArduPilot's sources are built by waf and cannot depend on a header generated during
the Pico-SDK CMake build. The regeneration command is in the comments at the top of
`dshot.pio`.

Verified with a Saleae DShot decoder at DShot600 and DShot300 simultaneously:
throttle values and checksums decoded as sent, with CRC passing on every channel, and
plain PWM channels continued to run undisturbed on their slices alongside.

`send_dshot_command()` is implemented. Values 0-47 of the throttle field are commands
rather than throttle, and an ESC only reads them as commands when the telemetry
request bit is set, so a command frame differs from a throttle frame in more than its
value. Commands are queued, because callers send them in runs such as a sequence of
beeps, and each is repeated for a number of frames because an ESC only acts on a
command it sees several times. While a command is running, channels it is not aimed at
are sent a zero throttle frame rather than nothing, so their ESCs stay armed. Once
armed, only commands flagged `priority` are accepted, so a beep request cannot
interrupt the throttle stream in flight.

Frames are sent from the 1kHz timer process, so `get_dshot_period_us()` reports 1000
and `set_dshot_period()` is not honoured; tying the rate to the vehicle loop would
mean replacing that timer process.

### Bidirectional DShot

Bidirectional DShot turns each output into a half duplex link: the frame is sent
inverted, the flight controller then stops driving the line, and the ESC answers with
its commutation period. `set_bidir_dshot_mask()` moves the channels named by
`SERVO_BLH_BDMASK` onto a second PIO program, `bdshot`, which sends the frame and
samples the answer without the CPU seeing an edge. Because PIO can turn a pin around
itself, any output can do this - there is no equivalent of the STM32 restriction to
particular timer channels.

The bit rates are what shapes that program. The response runs at 5/4 of the frame bit
rate, and one state machine has to clock both, so the frame bit is 40 cycles rather
than the 8 the unidirectional program uses: 40 is the smallest number that both
divides into eighths, for the 37.5%/75% bit encoding, and leaves the response bit a
whole number of cycles at 32. Sampling starts half a bit after the ESC pulls the line
down and then free runs, which the GCR encoding permits because it never leaves the
line without a transition for long.

Nothing in the program bounds its wait for a response, because a counter long enough
for a silent ESC will not fit in a `set` instruction. Instead the driver looks for an
answer one frame period after the frame went out, and puts a state machine still
waiting at that point back to the top of its program, so a missing or unpowered ESC
costs an error count rather than stopping the output.

What comes back is a 12 bit commutation period rather than a speed: a 9 bit mantissa
shifted up by a 3 bit exponent, in microseconds. `AP_ESC_Telem` wants motor RPM, so
`SERVO_BLH_POLES` divides it down. The decode itself is the usual one - the wire
encodes a `1` as a transition rather than as a level, so the sampled levels are
converted back to transitions before the four GCR quintets and the inverted checksum
are read off, using the same table BLHeli and betaflight use.

Two parts of the encoding differ from plain DShot and are easy to miss when reading a
capture: the whole waveform is inverted, so the line idles high and a bit starts low,
and the frame's checksum is inverted too. A DShot decoder set to the unidirectional
protocol will show a capture of this as CRC failures rather than as nothing.

The decode is exercised on the host rather than only on the bench, because it is
portable arithmetic and the failure mode - a table index or a shift in the wrong
direction - produces plausible looking eRPM rather than an obvious break. Encoding all
4096 representable telemetry values the way an ESC would and decoding them back covers
it in a way a bench run with one motor speed does not.

Verified on the bench with a logic analyzer, with no ESC attached: inverted frames
decode with passing checksums against a decoder set to bidirectional DShot600, and
they keep coming at 1kHz. That second half is the useful part of the test - nothing
ever answers, so every frame goes through the re-arm path, and a broken one would
show as a single frame per channel followed by silence. The receive half is covered
by the host test above rather than on the bench, and still wants a real ESC to
confirm end to end.

Not yet implemented: extended DShot telemetry (EDT), where an ESC reuses the exponent
field to send temperature, voltage and current. Those frames currently decode as eRPM.
The TX FIFO is deliberately left unjoined so that the RX FIFO stays available, which
is what makes the response path possible at all.

## Threading

FreeRTOS runs in SMP mode across both cores. Threads created by the HAL are pinned:
the main loop, timers and UARTs on core 0, and IO and storage on core 1. Threads
created through `hal.scheduler->thread_create()` are left unpinned for the scheduler
to place. Storage has its own thread because a flash program stops both cores, so
running it on the IO thread would delay IO procs behind every parameter save.

The kernel comes from Raspberry Pi's FreeRTOS-Kernel fork rather than upstream, as
the RP2350 ports are only in that fork.

## Toolchain

Builds with ArduPilot's usual `arm-none-eabi` 10-2020-q4 toolchain; no special
toolchain is needed.

If you do switch toolchains, note that CMake caches the compiler it was
configured with inside the build directory, so run `./waf distclean` afterwards.
A plain rebuild keeps the previously built objects, and linking objects from two
toolchain versions fails with an unhelpful `BFD ... assertion fail
.../elf32-arm.c` rather than a readable error.

## Building

```sh
# fetch dependencies into modules/ (neither is a tracked submodule)
Tools/scripts/rp2350_get_pico_sdk.sh
Tools/scripts/rp2350_get_freertos.sh

./waf configure --board rp2350generic
./waf copter
```

`PICO_SDK_PATH` and `FREERTOS_KERNEL_PATH` may be set to use existing checkouts.

The output image is `build/rp2350generic/pico-sdk_build/ardupilot.uf2`.

To flash, hold BOOTSEL while plugging the board in, then copy the `.uf2` onto the
mass-storage device the board presents (`/Volumes/RP2350` on macOS). The board
reboots into the new firmware automatically. The `cp` reports `Device not configured`,
which is normal - the board reboots part way through the write.

Holding the button is only needed for a board that is not already running this
firmware. Once it is, opening the USB CDC port at 1200 baud and closing it reboots the
board into BOOTSEL by itself, which makes the whole edit/flash/capture cycle
hands free.

## Testing on hardware

Each driver is exercised by a standalone program that builds in seconds rather than
the minutes a vehicle takes, and that fails in one subsystem rather than in all of
them at once. Several of ArduPilot's shared examples work here unchanged:

```sh
./waf --targets examples/Printf      # console
./waf --targets examples/BinarySem   # threads and semaphores
./waf --targets examples/UART_test   # serial ports, with a TX-RX loopback jumper
./waf --targets examples/StorageTest # parameter storage
./waf --targets examples/FlashTest   # the AP_FlashStorage layout
```

The rest live in `examples/` in this directory, because the shared ones assume things
that are not true here - `examples/AnalogIn` walks pin numbers 0-15, and
`examples/RCOutput` drives every channel with the same value, which cannot show that a
given channel reaches the GPIO it claims to. These build the same way, and are built
only for RP2350 boards:

```sh
./waf --targets examples/RP2350_AnalogIn  # ADC, with a jumper to 3V3 or AGND
./waf --targets examples/RP2350_RCOut     # PWM outputs, with a logic analyzer
./waf --targets examples/RP2350_DShot     # DShot, commands and eRPM
```

None of them need editing to change what they test, which is the point: a bench
session should not carry local modifications that have to be remembered and reverted
afterwards. They discover the board's pins and channels through the `AP_HAL`
interface - `valid_analog_pin()`, and `get_freq()` returning zero for an output that
does not exist - so they follow a board's `hwdef.dat` rather than restating it.

`RP2350_DShot` is driven from the console, so that changing one variable does not mean
reflashing and losing the analyzer setup. Connect to the USB CDC port at any baud rate
and press `?` for the key list:

```sh
screen /dev/cu.usbmodem*          # or any serial terminal
```

It asks which DShot rate to use during its first five seconds and then cannot be
changed, because moving a channel from a PWM slice to a PIO state machine is one way
in this driver. Everything else - arming, throttle, beep commands, and bidirectional
mode - is switchable while it runs.

Two things worth knowing before reading a capture. **A decoder has to be told the same
rate and encoding the board is sending**, and bidirectional DShot inverts both the
waveform and the frame checksum, so a decoder left on plain DShot reports CRC failures
rather than nothing at all. And with no ESC attached, bidirectional mode reports 0 eRPM
at a 100% error rate on every channel: that is the expected result, and the useful part
of the test is that frames keep going out at 1kHz anyway, which is what exercises the
re-arm path.

## Layout

| Path | Purpose |
|---|---|
| `hwdef/<board>/hwdef.dat` | per-board configuration |
| `examples/` | bench programs, built only for RP2350 boards |
| `hwdef/scripts/rp2350_hwdef.py` | generates `hwdef.h` from `hwdef.dat` |
| `targets/rp2350/CMakeLists.txt` | Pico-SDK CMake project that links the vehicle library |
| `targets/rp2350/FreeRTOSConfig.h` | FreeRTOS SMP configuration |
| `malloc.cpp` | locking, zeroing allocator wrappers |
| `Tools/ardupilotwaf/rp2350.py` | waf glue driving the CMake build |
