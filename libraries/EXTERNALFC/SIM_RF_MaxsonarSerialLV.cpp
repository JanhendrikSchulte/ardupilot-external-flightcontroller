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
  Simulator for the Maxsonar Serial LV rangefinder
*/

#include "SIM_RF_MaxsonarSerialLV.h"

#include <GCS_MAVLink/GCS.h>
#include <stdio.h>

using namespace EXTERNALFC;

uint32_t RF_MaxsonarSerialLV::packet_for_alt(uint16_t alt_cm, uint8_t *buffer, uint8_t buflen)
{
    const float inches = alt_cm / 2.54f;
    return snprintf((char*)buffer, buflen, "%u\r", (unsigned)inches);
}
