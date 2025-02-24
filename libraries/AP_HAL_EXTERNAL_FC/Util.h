#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_EXTERNAL_FC.h"

namespace EXTERNAL_FC {
	class Util : public AP_HAL::Util {
		/*
      get/set system clock in UTC microseconds
     */
    	void set_hw_rtc(uint64_t time_utc_usec) override;
    	uint64_t get_hw_rtc() const override;
	};	
}
