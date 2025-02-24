#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_EXTERNALFC_Namespace.h"
#include "AP_HAL_EXTERNALFC.h"
#include "Semaphores.h"
#include "ToneAlarm_SF.h"
#include <AP_Logger/AP_Logger_config.h>

#if !defined(__CYGWIN__) && !defined(__CYGWIN64__)
#include <sys/types.h>
#include <signal.h>
#endif

class HALEXTERNALFC::Util : public AP_HAL::Util {
public:
    Util(SITL_State *_sitlState) :
        sitlState(_sitlState) {}
    
    /**
       how much free memory do we have in bytes. 
     */
    uint32_t available_memory(void) override {
        // SITL is assumed to always have plenty of memory. Return 512k for now
        return 512*1024;
    }

    // get path to custom defaults file for AP_Param
    const char* get_custom_defaults_file() const override {
        return sitlState->defaults_path;
    }

    /**
       return commandline arguments, if available
     */
    void commandline_arguments(uint8_t &argc, char * const *&argv) override;
    
    uint64_t get_hw_rtc() const override;
    void set_hw_rtc(uint64_t time_utc_usec) override { /* fail silently */ }


    bool get_system_id(char buf[50]) override;
    bool get_system_id_unformatted(uint8_t buf[], uint8_t &len) override;
    void dump_stack_trace();

#ifdef WITH_SITL_TONEALARM
    bool toneAlarm_init(uint8_t types) override { return _toneAlarm.init(); }
    void toneAlarm_set_buzzer_tone(float frequency, float volume, uint32_t duration_ms) override {
        _toneAlarm.set_buzzer_tone(frequency, volume, duration_ms);
    }
#endif

    // return true if the reason for the reboot was a watchdog reset
    bool was_watchdog_reset() const override { return getenv("SITL_WATCHDOG_RESET") != nullptr; }

#if !defined(HAL_BUILD_AP_PERIPH)
    enum safety_state safety_switch_state(void) override;
    void set_cmdline_parameters() override;
#endif

    bool trap() const override {
#if defined(__CYGWIN__) || defined(__CYGWIN64__)
        return false;
#else
        if (kill(0, SIGTRAP) == -1) {
            return false;
        }
        return true;
#endif
    }

    void init(int argc, char *const *argv) {
        saved_argc = argc;
        saved_argv = argv;
    }

    // fills data with random values of requested size
    bool get_random_vals(uint8_t* data, size_t size) override;

private:
    SITL_State *sitlState;

#ifdef WITH_SITL_TONEALARM
    static ToneAlarm_SF _toneAlarm;
#endif

    int saved_argc;
    char *const *saved_argv;

#if HAL_UART_STATS_ENABLED
    // request information on uart I/O
    void uart_info(ExpandingString &str) override;

#if HAL_LOGGING_ENABLED
    // Log UART message for each serial port
    void uart_log() override;
#endif
#endif // HAL_UART_STATS_ENABLED

private:
#if HAL_UART_STATS_ENABLED
    // UART stats tracking helper
    struct uart_stats {
        AP_HAL::UARTDriver::StatsTracker serial[AP_HAL::HAL::num_serial];
        uint32_t last_ms;
    };
    uart_stats sys_uart_stats;
#if HAL_LOGGING_ENABLED
    uart_stats log_uart_stats;
#endif
#endif // HAL_UART_STATS_ENABLED
};
