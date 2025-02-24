#pragma once

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_SITL_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_Proximity.h"

#include "AP_Proximity_Backend.h"
#include <SITL/SITL.h>
#include <AP_Common/Location.h>

class AP_Proximity_SITL : public AP_Proximity_Backend
{

public:
    // constructor
    AP_Proximity_SITL(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state, AP_Proximity_Params& _params);

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override;
    float distance_min() const override;

    // get distance upwards in meters. returns true on success
    bool get_upward_distance(float &distance) const override;

private:
    SITL::SIM *sitl;
    AP_Float *fence_alt_max;
    Location current_loc;

    // get distance in meters to fence in a particular direction in degrees (0 is forward, angles increase in the clockwise direction)
    bool get_distance_to_fence(float angle_deg, float &distance) const;

};

#endif // AP_PROXIMITY_SITL_ENABLED
