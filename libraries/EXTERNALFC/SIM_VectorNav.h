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
  simulate serial VectorNav AHRS

  Usage example:
     SERIAL5_PROTOCOL = 36
     AHRS_EKF_TYPE = 11
     EAHRS_TYPE=1

     sim_vehicle.py -D --console --map -A --serial5=sim:VectorNav
*/

#pragma once

#include "SIM_Aircraft.h"

#include <EXTERNALFC/EXTERNALFC.h>
#include "SIM_SerialDevice.h"

namespace EXTERNALFC {

class VectorNav : public SerialDevice {
public:

    VectorNav();

    // update state
    void update(void);

private:
    uint32_t last_imu_pkt_us;
    uint32_t last_ekf_pkt_us;
    uint32_t last_gnss_pkt_us;

    void send_imu_packet();
    void send_ins_ekf_packet();
    void send_ins_gnss_packet();
    void nmea_printf(const char *fmt, ...);
};

}
