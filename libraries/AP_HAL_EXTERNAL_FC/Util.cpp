
#include "Util.h"

namespace EXTERNAL_FC
{
	uint64_t Util::get_hw_rtc() const
	{
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		const uint64_t seconds = ts.tv_sec;
		const uint64_t nanoseconds = ts.tv_nsec;
		return (seconds * 1000000ULL + nanoseconds / 1000ULL);
	}

	void Util::set_hw_rtc(uint64_t time_utc_usec)
	{
// don't reset the HW clock time on people's laptops.
#if CONFIG_HAL_BOARD != HAL_BOARD_EXTERNAL_FC
		timespec ts;
		ts.tv_sec = time_utc_usec / 1000000ULL;
		ts.tv_nsec = (time_utc_usec % 1000000ULL) * 1000ULL;
		clock_settime(CLOCK_REALTIME, &ts);
#endif
	}
}
