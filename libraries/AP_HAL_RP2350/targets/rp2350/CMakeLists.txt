cmake_minimum_required(VERSION 3.13)

# PICO_SDK_PATH is passed in by Tools/ardupilotwaf/rp2350.py
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(ardupilot C CXX ASM)

# ArduPilot's libraries/AP_Common/c++.cpp already provides __wrap_malloc (which,
# unlike the SDK's, zeroes the returned memory) and the global new/delete
# operators. Declaring these targets up front makes the SDK skip its own
# conflicting versions, via the "if (NOT TARGET ...)" guards in
# src/rp2_common/pico_malloc and src/rp2_common/pico_cxx_options.
add_library(pico_malloc_headers INTERFACE)
add_library(pico_malloc INTERFACE)

add_library(pico_cxx_options_headers INTERFACE)
add_library(pico_cxx_options INTERFACE)
# mirror the SDK's own defaults for these, minus its new_delete.cpp
target_include_directories(pico_cxx_options SYSTEM INTERFACE
    ${PICO_SDK_PATH}/src/rp2_common/pico_cxx_options/include)
target_compile_definitions(pico_cxx_options INTERFACE PICO_CXX_ENABLE_EXCEPTIONS=0)
target_compile_options(pico_cxx_options INTERFACE
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-unwind-tables>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
)

pico_sdk_init()

# FreeRTOS SMP kernel; FREERTOS_KERNEL_PATH is passed in by rp2350.py.
# FREERTOS_CONFIG_FILE_DIRECTORY tells the port where FreeRTOSConfig.h lives.
set(FREERTOS_CONFIG_FILE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})
add_subdirectory(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2350_ARM_NTZ FreeRTOS-Kernel)

# the Pico-SDK appends the .elf suffix itself, giving ardupilot.elf
set(elf_file ardupilot)

# Pico-SDK's crt0 calls main() directly, and ArduPilot's AP_HAL_MAIN() macro
# defines main(), so the vehicle's own static library provides the entry
# point. This stub only exists to give CMake a source file for the target;
# it is C++ so the executable links as C++ and picks up libsupc++ (which
# provides std::nothrow and __cxa_pure_virtual).
add_executable(${elf_file} stub.cpp)

if(NOT DEFINED ARDUPILOT_CMD)
    set(ARDUPILOT_CMD "none")
endif()

message("ARDUPILOT_CMD=${ARDUPILOT_CMD}")
message("WAF_BUILD_TARGET=${WAF_BUILD_TARGET}")

# examples/tools are small standalone programs (libraries/*/examples/*) used to
# exercise one driver at a time on real hardware
string(REGEX MATCH "^(examples|tool)/" IS_EXAMPLE "${WAF_BUILD_TARGET}")

if(IS_EXAMPLE)
    string(REPLACE "/" ";" A ${WAF_BUILD_TARGET})
    list(GET A 0 EXAMPLE_BASE)
    list(GET A 1 EXAMPLE_NAME)
    message("Building ${EXAMPLE_BASE} ${EXAMPLE_NAME}")
    target_link_libraries(${elf_file} "${ARDUPILOT_LIB}/${EXAMPLE_BASE}/lib${EXAMPLE_NAME}.a")
    target_link_libraries(${elf_file} "${ARDUPILOT_LIB}/libap.a")
elseif(${ARDUPILOT_CMD} STREQUAL "copter")
    message("Building for copter")
    target_link_libraries(${elf_file} "${ARDUPILOT_BIN}/libarducopter.a")
    target_link_libraries(${elf_file} "${ARDUPILOT_LIB}/libArduCopter_libs.a")
elseif(${ARDUPILOT_CMD} STREQUAL "plane")
    message("Building for plane")
    target_link_libraries(${elf_file} "${ARDUPILOT_BIN}/libarduplane.a")
    target_link_libraries(${elf_file} "${ARDUPILOT_LIB}/libArduPlane_libs.a")
elseif(${ARDUPILOT_CMD} STREQUAL "rover")
    message("Building for rover")
    target_link_libraries(${elf_file} "${ARDUPILOT_BIN}/libardurover.a")
    target_link_libraries(${elf_file} "${ARDUPILOT_LIB}/libRover_libs.a")
endif()

target_link_libraries(${elf_file}
                FreeRTOS-Kernel
                FreeRTOS-Kernel-Heap3
                pico_stdlib
                pico_stdio_usb
                pico_unique_id
                pico_multicore
                pico_flash
                hardware_adc
                hardware_dma
                hardware_flash
                hardware_i2c
                hardware_pio
                hardware_pwm
                hardware_spi
                hardware_watchdog
                )

# AP_HAL_RP2350/malloc.cpp supplies these; see the comment above about
# pico_malloc. All four are wrapped so the whole allocator is locked.
target_link_options(${elf_file} PRIVATE
    "-Wl,--wrap=malloc"
    "-Wl,--wrap=calloc"
    "-Wl,--wrap=realloc"
    "-Wl,--wrap=free"
)

pico_add_extra_outputs(${elf_file})

# Publish the SDK's include paths and compile definitions so the waf build
# can compile ArduPilot's own sources against Pico-SDK headers with the same
# platform configuration (PICO_RP2350, board selection, TinyUSB config, ...).
add_custom_target(showinc ALL
        COMMAND echo
        "$<TARGET_PROPERTY:${elf_file},INCLUDE_DIRECTORIES>"
        > includes.list
    COMMAND echo
        "$<TARGET_PROPERTY:${elf_file},COMPILE_DEFINITIONS>"
        > defines.list
    VERBATIM
    BYPRODUCTS includes.list defines.list
    COMMAND_EXPAND_LISTS
)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
