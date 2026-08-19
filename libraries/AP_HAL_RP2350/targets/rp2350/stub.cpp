/*
  Placeholder translation unit for the Pico-SDK CMake executable target,
  which requires at least one source file. ArduPilot's main() comes from the
  vehicle static library linked in by CMakeLists.txt (see AP_HAL_MAIN() in
  libraries/AP_HAL/AP_HAL_Main.h).

  This must not become an empty translation unit. GCC emits the ARM build
  attributes for a file from the code it generates, so an empty one is
  tagged Tag_CPU_name "Cortex-M33" with no Tag_FP_arch, rather than
  "8-M.MAIN" with FPv5/FP-D16 like every other object here. Merging that
  odd-one-out into the link makes the arm-none-eabi 10-2020-q4 linker
  (binutils 2.35.1) fail an internal assertion in elf32-arm.c instead of
  producing a firmware image. A data-only definition is not enough; the
  attributes only come out right if the file generates code.
*/

extern "C" const char *ap_hal_rp2350_name(void);

extern "C" const char *ap_hal_rp2350_name(void)
{
    return "AP_HAL_RP2350";
}
