#pragma once

#include "AP_Beacon_Backend.h"

#if AP_BEACON_SITL_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_EXTERNALFC

#include <EXTERNALFC/EXTERNALFC.h>

class AP_Beacon_SITL : public AP_Beacon_Backend
{

public:
    // constructor
    AP_Beacon_SITL(AP_Beacon &frontend);

    // return true if sensor is basically healthy (we are receiving data)
    bool healthy() override;

    // update
    void update() override;

private:
	EXTERNALFC::SIM *sitl;
    uint8_t next_beacon;
    uint32_t last_update_ms;
};

#endif // AP_BEACON_SITL_ENABLED
