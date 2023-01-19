#pragma once

#include <atomic>
#include "../std_type.hpp"

namespace crs_lib::MotorDriver::RoboMaster
{
	struct SpeedControllerMessage final
	{
		std::atomic<i16> target_currents[4];

		u64 get_packed_data() const
		{
			return target_currents[0] | target_currents[1] << 2 | target_currents[2] << 4 | target_currents[3] << 6;
		}
	};
}