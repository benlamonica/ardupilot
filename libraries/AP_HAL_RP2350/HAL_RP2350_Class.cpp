#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>

#include <FreeRTOS.h>
#include <task.h>

#include "HAL_RP2350_Class.h"
#include "Scheduler.h"
#include "GPIO.h"
#include "UARTDriver.h"
#include "Util.h"

// Bring-up: only the serial ports, GPIO and Scheduler are real. Every other
// peripheral is wired to AP_HAL_Empty's no-op stub until its driver lands
// (see the RP2350 port plan, Phase 2).
static RP2350::UARTDriver cons(RP2350::UARTDriver::USB_CONSOLE);
#ifdef HAL_RP2350_UART_DEVICES
// the RP2350 has exactly two UARTs; each is an index into the table the
// board's RP2350_SERIAL lines generate, and an index the board does not
// declare leaves that port permanently uninitialised
static RP2350::UARTDriver serial1Driver(0);
static RP2350::UARTDriver serial2Driver(1);
#else
static Empty::UARTDriver serial1Driver;
static Empty::UARTDriver serial2Driver;
#endif
static Empty::UARTDriver serial3Driver;
static Empty::UARTDriver serial4Driver;
static Empty::UARTDriver serial5Driver;
static Empty::UARTDriver serial6Driver;
static Empty::UARTDriver serial7Driver;
static Empty::UARTDriver serial8Driver;
static Empty::UARTDriver serial9Driver;

static Empty::I2CDeviceManager i2cDeviceManager;
static Empty::SPIDeviceManager spiDeviceManager;
static Empty::AnalogIn analogIn;
static Empty::Storage storageDriver;
static RP2350::GPIO gpioDriver;
static Empty::RCInput rcinDriver;
static Empty::RCOutput rcoutDriver;
static RP2350::Scheduler schedulerInstance;
static RP2350::Util utilInstance;
static Empty::OpticalFlow opticalFlowDriver;
static Empty::Flash flashDriver;

HAL_RP2350::HAL_RP2350() :
    AP_HAL::HAL(
        &cons,          // Console/mavlink
        &serial1Driver, // Telem 1
        &serial2Driver, // Telem 2
        &serial3Driver, // GPS 1
        &serial4Driver, // GPS 2
        &serial5Driver, // Extra 1
        &serial6Driver, // Extra 2
        &serial7Driver, // Extra 3
        &serial8Driver, // Extra 4
        &serial9Driver, // Extra 5
        &i2cDeviceManager,
        &spiDeviceManager,
        nullptr,        // wspi
        &analogIn,
        &storageDriver,
        &cons,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        &opticalFlowDriver,
        &flashDriver,
        nullptr         // can_ifaces
    )
{}

void HAL_RP2350::run(int argc, char* const argv[], Callbacks* callbacks) const
{
    schedulerInstance.set_callbacks(callbacks);

    // creates the APM threads; the main thread runs callbacks->setup() and
    // then loops callbacks->loop()
    scheduler->init();

    // hands both cores to FreeRTOS and does not return
    vTaskStartScheduler();

    AP_HAL::panic("FreeRTOS scheduler exited");
}
