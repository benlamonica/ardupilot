# ArduPilot port to the RP2350 (Raspberry Pi Pico 2)

This HAL wraps the Raspberry Pi Pico-SDK, in the same way `AP_HAL_ESP32` wraps
Espressif's ESP-IDF: the vendor SDK supplies the low-level peripheral drivers and
this directory adapts them to the `AP_HAL` interfaces.

**Status: early bring-up.** The build produces a complete firmware image, and runs
under FreeRTOS on both Cortex-M33 cores, but only the Scheduler, Semaphores, GPIO,
USB console UART and Util are implemented. Every other peripheral is wired to an
`AP_HAL_Empty` stub, so there are no sensors, no motor output and no parameter
storage yet. It has not been flight tested, and it is not airworthy.

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
