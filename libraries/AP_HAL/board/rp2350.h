#pragma once

#include <hwdef.h>

#define HAL_BOARD_NAME "RP2350"
#define HAL_CPU_CLASS HAL_CPU_CLASS_150
#define HAL_MEM_CLASS HAL_MEM_CLASS_500

#define HAL_WITH_DRONECAN 0
#define HAL_WITH_UAVCAN 0
#define HAL_NUM_CAN_IFACES 0
#define HAL_MAX_CAN_PROTOCOL_DRIVERS 0

#define HAL_HAVE_SAFETY_SWITCH 0
#define HAL_HAVE_BOARD_VOLTAGE 0
#define HAL_HAVE_SERVO_VOLTAGE 0
#define HAL_WITH_IO_MCU 0

#define HAL_STORAGE_SIZE (16384)

#ifndef HAL_PROGRAM_SIZE_LIMIT_KB
#define HAL_PROGRAM_SIZE_LIMIT_KB 2048
#endif

// RP2350's Cortex-M33 FPU is single-precision only, same as the STM32F4
// boards this HAL_MEM_CLASS/HAL_CPU_CLASS tier already covers. These must
// be set before including Semaphores.h below, which pulls in
// AP_HAL_Macros.h and evaluates HAL_WITH_EKF_DOUBLE.
#define HAL_HAVE_HARDWARE_DOUBLE 0
#define HAL_WITH_EKF_DOUBLE HAL_HAVE_HARDWARE_DOUBLE

#ifdef __cplusplus
#include <AP_HAL_RP2350/Semaphores.h>
#define HAL_Semaphore RP2350::Semaphore
#define HAL_BinarySemaphore RP2350::BinarySemaphore
#endif

#define __LITTLE_ENDIAN  1234
#define __BYTE_ORDER     __LITTLE_ENDIAN

#define NUM_SERVO_CHANNELS 16

// Phase 1 bring-up has no sensors/CAN/networking wired up yet; keep the
// build small and disable the same class of optional subsystems the other
// early/constrained HAL (ESP32) disables by default. These can be turned
// back on per-feature as real drivers land.
#define AP_EXTERNAL_AHRS_ENABLED 0
#define HAL_GENERATOR_ENABLED 0
#define HAL_MOUNT_ENABLED 0
#define AP_CAMERA_ENABLED 0
#define HAL_SOARING_ENABLED 0
#define AP_TERRAIN_AVAILABLE 0
#define HAL_ADSB_ENABLED 0
#define HAL_BUTTON_ENABLED 0
#define AP_GRIPPER_ENABLED 0
#define AP_LANDINGGEAR_ENABLED 0
#define AP_AVOIDANCE_ENABLED 0
#define AP_FENCE_ENABLED 0
#define MODE_FOLLOW_ENABLED 0
#define AP_OAPATHPLANNER_ENABLED 0
#define HAL_QUADPLANE_ENABLED 0
#define HAL_GYROFFT_ENABLED 0

// no filesystem backend is wired up yet, and scripting requires one
#define AP_SCRIPTING_ENABLED 0
#define AP_OPTICALFLOW_ENABLED 0
#define AP_RPM_ENABLED 0
#define AP_ICENGINE_ENABLED 0
#define AP_ADVANCEDFAILSAFE_ENABLED 0

#ifndef AP_NOTIFY_BUZZER_ENABLED
#define AP_NOTIFY_BUZZER_ENABLED 0
#endif
