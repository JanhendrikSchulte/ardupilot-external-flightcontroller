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
/*
  Simulator for the NMEA serial rangefinder

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --serial5=sim:rf_mavlink --speedup=1

param set SERIAL5_PROTOCOL 1
param set RNGFND1_TYPE 10
graph RANGEFINDER.distance
graph GLOBAL_POSITION_INT.relative_alt/1000-RANGEFINDER.distance
reboot

arm throttle
rc 3 1600
*/

#pragma once

#include "SIM_SerialRangeFinder.h"

namespace EXTERNALFC {

class RF_MAVLink : public SerialRangeFinder {
public:

    uint32_t packet_for_alt(uint16_t alt_cm, uint8_t *buffer, uint8_t buflen) override;

private:
    mavlink_status_t mav_status;
};

}
