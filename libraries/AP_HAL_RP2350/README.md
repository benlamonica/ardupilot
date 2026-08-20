# ArduPilot port to the RP2350 (Raspberry Pi Pico 2)

This HAL wraps the Raspberry Pi Pico-SDK, in the same way `AP_HAL_ESP32` wraps
Espressif's ESP-IDF: the vendor SDK supplies the low-level peripheral drivers and
this directory adapts them to the `AP_HAL` interfaces.

**Status: early bring-up.** The build produces a complete firmware image, and runs
under FreeRTOS on both Cortex-M33 cores, but only the Scheduler, Semaphores, GPIO,
serial ports and Util are implemented. Every other peripheral is wired to an
`AP_HAL_Empty` stub, so there are no sensors, no motor output and no parameter
storage yet. It has not been flight tested, and it is not airworthy.

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

## Threading

FreeRTOS runs in SMP mode across both cores. Threads created by the HAL are pinned:
the main loop, timers and UARTs on core 0, and IO on core 1. Threads created through
`hal.scheduler->thread_create()` are left unpinned for the scheduler to place.

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

Individual drivers are exercised with the standalone example programs, which build
much faster than a full vehicle and isolate one subsystem:

```sh
./waf --targets examples/Printf      # console
./waf --targets examples/BinarySem   # threads and semaphores
./waf --targets examples/UART_test   # serial ports, with a TX-RX loopback jumper
```

The output image is `build/rp2350generic/pico-sdk_build/ardupilot.uf2`.

To flash, hold BOOTSEL while plugging the board in, then copy the `.uf2` onto the
mass-storage device the board presents (`/Volumes/RP2350` on macOS). The board
reboots into the new firmware automatically.

## Layout

| Path | Purpose |
|---|---|
| `hwdef/<board>/hwdef.dat` | per-board configuration |
| `hwdef/scripts/rp2350_hwdef.py` | generates `hwdef.h` from `hwdef.dat` |
| `targets/rp2350/CMakeLists.txt` | Pico-SDK CMake project that links the vehicle library |
| `targets/rp2350/FreeRTOSConfig.h` | FreeRTOS SMP configuration |
| `malloc.cpp` | locking, zeroing allocator wrappers |
| `Tools/ardupilotwaf/rp2350.py` | waf glue driving the CMake build |
