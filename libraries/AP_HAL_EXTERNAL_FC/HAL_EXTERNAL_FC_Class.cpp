#include "HAL_EXTERNAL_FC_Class.h"

#include <assert.h>
#include <signal.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RCOutput_Tap.h>
#include <AP_HAL/utility/getopt_cpp.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_HAL_Empty/AP_HAL_Empty_Private.h>
#include <AP_Module/AP_Module.h>

#include "Scheduler.h"
//#include "Storage.h"
#include "Util.h"

using namespace EXTERNAL_FC;


static Empty::AnalogIn analogIn;
static Empty::GPIO gpioDriver;
static Empty::I2CDeviceManager i2c_mgr_instance;
static Empty::SPIDeviceManager spi_mgr_instance;
static Empty::UARTDriver serial0Driver;
static Empty::UARTDriver serial1Driver;
static Empty::UARTDriver serial2Driver;
static Empty::UARTDriver serial3Driver;
static Empty::UARTDriver uartDriver;

static Empty::Storage storageDriver;
static Util utilInstance;

static Empty::RCInput rcinDriver;
static Empty::RCOutput rcoutDriver;

static Scheduler schedulerInstance;

static Empty::OpticalFlow opticalFlow;

// TODO-TBU Conditional? why? 
#if HAL_WITH_DSP
  static Empty::DSP dspDriver;
#endif

static Empty::Flash flashDriver;
static Empty::WSPIDeviceManager wspi_mgr_instance;

HAL_EXTERNAL_FC::HAL_EXTERNAL_FC() :
    AP_HAL::HAL(
        &serial0Driver,
        &serial1Driver,
        &serial2Driver,
        &serial3Driver,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        &i2c_mgr_instance,
        &spi_mgr_instance,
        &wspi_mgr_instance,
        &analogIn,
        &storageDriver,
        &serial0Driver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance,
        &opticalFlow,
        &flashDriver,
#if HAL_WITH_DSP
        &dspDriver,
#endif
#if HAL_NUM_CAN_IFACES
        (AP_HAL::CANIface**)canDrivers
#else
        nullptr
#endif
        )
{}

void HAL_EXTERNAL_FC::run(int argc, char* const argv[], Callbacks* callbacks) const
{
#if AP_MODULE_SUPPORTED
    const char *module_path = AP_MODULE_DEFAULT_DIRECTORY;
#endif

    enum long_options {
        CMDLINE_SERIAL0 = 1, // must be in 0-9 order and numbered consecutively
        CMDLINE_SERIAL1,
        CMDLINE_SERIAL2,
        CMDLINE_SERIAL3,
        CMDLINE_SERIAL4,
        CMDLINE_SERIAL5,
        CMDLINE_SERIAL6,
        CMDLINE_SERIAL7,
        CMDLINE_SERIAL8,
        CMDLINE_SERIAL9,
    };

    //int opt;
    const struct GetOptLong::option options[] = {
        {"uartA",         true,  0, 'A'},
        {"uartB",         true,  0, 'B'},
        {"uartC",         true,  0, 'C'},
        {"uartD",         true,  0, 'D'},
        {"uartE",         true,  0, 'E'},
        {"uartF",         true,  0, 'F'},
        {"uartG",         true,  0, 'G'},
        {"uartH",         true,  0, 'H'},
        {"uartI",         true,  0, 'I'},
        {"uartJ",         true,  0, 'J'},
        {"serial0",       true,  0, CMDLINE_SERIAL0},
        {"serial1",       true,  0, CMDLINE_SERIAL1},
        {"serial2",       true,  0, CMDLINE_SERIAL2},
        {"serial3",       true,  0, CMDLINE_SERIAL3},
        {"serial4",       true,  0, CMDLINE_SERIAL4},
        {"serial5",       true,  0, CMDLINE_SERIAL5},
        {"serial6",       true,  0, CMDLINE_SERIAL6},
        {"serial7",       true,  0, CMDLINE_SERIAL7},
        {"serial8",       true,  0, CMDLINE_SERIAL8},
        {"serial9",       true,  0, CMDLINE_SERIAL9},
        {"log-directory",       true,  0, 'l'},
        {"terrain-directory",   true,  0, 't'},
        {"storage-directory",   true,  0, 's'},
        {"module-directory",    true,  0, 'M'},
        {"defaults",            true,  0, 'd'},
        {"cpu-affinity",        true,  0, 'c'},
        {"help",                false,  0, 'h'},
        {0, false, 0, 0}
    };

    GetOptLong gopt(argc, argv, "A:B:C:D:E:F:G:H:I:J:l:t:s:he:SM:c:",
                    options);

    // NOTE: signal handlers are only set before the main loop, so
    // that if anything before the loops hangs, the default signals
    // can still stop the process proprely, although without proper
    // teardown.
    // This isn't perfect, but still prevents an unkillable process.

    scheduler->init();
    gpio->init();
    rcout->init();
    rcin->init();
    serial(0)->begin(115200);
    analogin->init();
    //utilInstance.init(argc+gopt.optind-1, &argv[gopt.optind-1]);

    // NOTE: See commit 9f5b4ffca ("AP_HAL_Linux_Class: Correct
    // deadlock, and infinite loop in setup()") for details about the
    // order of scheduler initialize and setup on Linux.
    scheduler->set_system_initialized();

    // possibly load external modules
#if AP_MODULE_SUPPORTED
    if (module_path != nullptr) {
        AP_Module::init(module_path);
    }
#endif

#if AP_MODULE_SUPPORTED
    AP_Module::call_hook_setup_start();
#endif
    callbacks->setup();
#if AP_MODULE_SUPPORTED
    AP_Module::call_hook_setup_complete();
#endif

    setup_signal_handlers();

    while (!_should_exit) {
        callbacks->loop();
    }

    // At least try to stop all PWM before shutting down
   /*  rcout->force_safety_on();
    rcin->teardown();
    I2CDeviceManager::from(i2c_mgr)->teardown();
    SPIDeviceManager::from(spi)->teardown();
    Scheduler::from(scheduler)->teardown(); */
}

void HAL_EXTERNAL_FC::setup_signal_handlers() const
{
    struct sigaction sa = { };

    sa.sa_flags = SA_NOCLDSTOP;
    sa.sa_handler = HAL_EXTERNAL_FC::exit_signal_handler;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);
}

HAL_EXTERNAL_FC hal_linux;

void HAL_EXTERNAL_FC::exit_signal_handler(int signum)
{
    hal_linux._should_exit = true;
}

const AP_HAL::HAL &AP_HAL::get_HAL()
{
    return hal_linux;
}

AP_HAL::HAL &AP_HAL::get_HAL_mutable()
{
    return hal_linux;
}
