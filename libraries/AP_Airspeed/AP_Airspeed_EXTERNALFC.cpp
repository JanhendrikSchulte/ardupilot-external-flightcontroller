#include "AP_Airspeed_EXTERNALFC.h"

#if AP_AIRSPEED_SITL_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_EXTERNALFC

#include <AP_Baro/AP_Baro.h>
#include <EXTERNALFC/EXTERNALFC.h>

// return the current differential_pressure in Pascal
bool AP_Airspeed_SITL::get_differential_pressure(float &pressure)
{
    const uint8_t _instance = get_instance();

    if (_instance >= AIRSPEED_MAX_SENSORS) {
        return false;
    }

    pressure = AP::sitl()->state.airspeed_raw_pressure[_instance];

    return true;
}

// get last temperature
bool AP_Airspeed_SITL::get_temperature(float &temperature)
{
    const uint8_t _instance = get_instance();

    if (_instance >= AIRSPEED_MAX_SENSORS) {
        return false;
    }

    const auto *sitl = AP::sitl();

    // this was mostly swiped from SIM_Airspeed_DLVR:
    const float sim_alt = sitl->state.altitude;

    // To Do: Add a sensor board temperature offset parameter
    temperature = AP_Baro::get_temperatureC_for_alt_amsl(sim_alt);

    return true;
}

#endif // AP_AIRSPEED_SITL_ENABLED
