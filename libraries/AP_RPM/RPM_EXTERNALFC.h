/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_RPM.h"

#if AP_RPM_SIM_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_EXTERNALFC

#include "RPM_Backend.h"
#include <EXTERNALFC/EXTERNALFC.h>

class AP_RPM_SITL : public AP_RPM_Backend
{
public:
    // constructor
    AP_RPM_SITL(AP_RPM &ranger, uint8_t instance, AP_RPM::RPM_State &_state);

    // update state
    void update(void) override;
private:
	EXTERNALFC::SIM *sitl;
    uint8_t instance;
};

#endif // AP_RPM_SIM_ENABLED
