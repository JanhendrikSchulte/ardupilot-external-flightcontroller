#pragma once

#define HAL_BOARD_NAME "Linux"
#define HAL_CPU_CLASS HAL_CPU_CLASS_1000
#define HAL_MEM_CLASS HAL_MEM_CLASS_1000
#define HAL_OS_SOCKETS 1

// 32768 possible
#define HAL_STORAGE_SIZE            16384
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE
#define HAL_FLASH_SECTOR_SIZE 128*1024
#define HAL_FLASH_MIN_WRITE_SIZE 32
#define HAL_FLASH_ALLOW_UPDATE 0


#define HAL_BOARD_LOG_DIRECTORY "logs"
#define HAL_BOARD_TERRAIN_DIRECTORY "terrain"
#define HAL_BOARD_STORAGE_DIRECTORY "."
#define HAL_INS_DEFAULT HAL_INS_NONE
#define HAL_BARO_DEFAULT HAL_BARO_NONE
// TODO-TBD Params file? current state: SITL
#define HAL_PARAM_DEFAULTS_PATH nullptr

#ifndef BOARD_FLASH_SIZE
#define BOARD_FLASH_SIZE 4096
#endif



// TODO-TBD Remove?
// #define AP_NOTIFY_GPIO_LED_RGB_ENABLED 1
#define AP_NOTIFY_GPIO_LED_RGB_RED_PIN    8  // these are set in SIM_PIN_MASK
#define AP_NOTIFY_GPIO_LED_RGB_GREEN_PIN  9
#define AP_NOTIFY_GPIO_LED_RGB_BLUE_PIN  10

// #define AP_NOTIFY_GPIO_LED_1_ENABLED 1
#define AP_NOTIFY_GPIO_LED_A_PIN          8  // these are set in SIM_PIN_MASK


#define HAL_HAVE_BOARD_VOLTAGE 1

// TODO-TBD 0 for linux, 1 for sitl?
#define HAL_HAVE_SAFETY_SWITCH 0

// TODO-TBD 0 for linux, 1 for sitl?
#define HAL_HAVE_SERVO_VOLTAGE 0

#ifndef HAL_HAVE_HARDWARE_DOUBLE
#define HAL_HAVE_HARDWARE_DOUBLE 1
#endif

#ifndef HAL_WITH_EKF_DOUBLE
#define HAL_WITH_EKF_DOUBLE HAL_HAVE_HARDWARE_DOUBLE
#endif


// only include if compiling C++ code
#ifdef __cplusplus
// allow for static semaphores
// TODO-TBD Namespace
#include <AP_HAL_EXTERNAL_FC/Semaphores.h>
#define HAL_Semaphore EXTERNAL_FC::Semaphore
#define HAL_BinarySemaphore Linux::BinarySemaphore
#endif

#ifndef HAL_NUM_CAN_IFACES
#define HAL_NUM_CAN_IFACES 0
#endif

#ifndef HAL_BOARD_STORAGE_DIRECTORY
#define HAL_BOARD_STORAGE_DIRECTORY "."
#endif



#ifndef HAL_CAN_DRIVER_DEFAULT
#define HAL_CAN_DRIVER_DEFAULT 0
#endif

#ifndef HAL_INS_RATE_LOOP
#define HAL_INS_RATE_LOOP 1
#endif

// TODO-TBD check if it helps, if shit goes wild
#ifndef HAL_GYROFFT_ENABLED
#define HAL_GYROFFT_ENABLED 0
#endif



#ifndef AP_UART_MONITOR_ENABLED
// TODO-TBD defaults to 1 in sitl, is it needed?
#define AP_UART_MONITOR_ENABLED 0
#endif

// TODO-TBD should probably be 0, but was 1 in sitl
#ifndef AP_FILTER_ENABLED
#define AP_FILTER_ENABLED 0
#endif

// TODO-TBD Shouldn't be needed
// #define HAL_SOLO_GIMBAL_ENABLED 1

// TODO-TBD Remove once i2cdevice is gone
#ifndef HAL_LINUX_I2C_BUS_MASK
    #define HAL_LINUX_I2C_BUS_MASK 0xFFFF
#endif

#ifndef HAL_LINUX_I2C_INTERNAL_BUS_MASK
    #define HAL_LINUX_I2C_INTERNAL_BUS_MASK 0xFFFF
#endif

#ifndef HAL_LINUX_I2C_EXTERNAL_BUS_MASK
    #define HAL_LINUX_I2C_EXTERNAL_BUS_MASK 0xFFFF
#endif
