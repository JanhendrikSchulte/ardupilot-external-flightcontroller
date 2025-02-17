/*
  SITL handling

  This simulates a rangefinder

 */

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_EXTERNALFC && !defined(HAL_BUILD_AP_PERIPH)

#include "AP_HAL_EXTERNALFC.h"
#include "AP_HAL_EXTERNALFC_Namespace.h"
#include "HAL_EXTERNALFC_Class.h"
#include "EXTERNALFC_State.h"
#include <EXTERNALFC/EXTERNALFC.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

using namespace EXTERNALFC;

// returns a voltage between 0V to 5V which should appear as the
// voltage from the sensor
float SITL_State::_sonar_pin_voltage() const
{
    // Use glitch defines as the probablility between 0-1 that any
    // given sonar sample will read as max distance
    if (!is_zero(_sitl->sonar_glitch) &&
        _sitl->sonar_glitch >= (rand_float() + 1.0f) / 2.0f) {
        // glitched
        return 5.0f;
    }

    const float altitude = sitl_model->rangefinder_range();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
    if (altitude == INFINITY) {
        return 5.0f;
    }
#pragma GCC diagnostic pop

    // Altitude in in m, scaler in meters/volt
    const float voltage = altitude / _sitl->sonar_scale;

    // constrain to 0-5V
    return constrain_float(voltage, 0.0f, 5.0f);
}

/*
  setup the rangefinder with new input
 */
void SITL_State::_update_rangefinder()
{
    sonar_pin_voltage = _sonar_pin_voltage();
}

#endif
