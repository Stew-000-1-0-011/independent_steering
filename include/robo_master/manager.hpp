#pragma once

#include <ros/ros.h>

#include <independent_steering/CanFrame.h>

#include "speed_controller_message.hpp"
#include "md.hpp"

namespace crs_lib::MotorDriver::RoboMaster
{
	class RoboMasterMdManager final
	{
		ros::Publisher speed_controller1_4_pub;
		ros::Publisher speed_controller5_8_pub;

		SpeedControllerMessage motor1_4{};
		SpeedControllerMessage motor5_8{};

	public:
		RoboMasterMdManager(ros::NodeHandle& nh):
			speed_controller1_4_pub{nh.advertise<independent_steering::CanFrame>("can" + std::to_string(0x200), 1)},
			speed_controller5_8_pub{nh.advertise<independent_steering::CanFrame>("can" + std::to_string(0x1FF), 1)}
		{}

		void publish()
		{
			independent_steering::CanFrame frame{};
			frame.dlc = 8;
			const u64 data1_4 = motor1_4.get_packed_data();
			std::memcpy(frame.data.data(), &data1_4, 8);
			speed_controller1_4_pub.publish(frame);

			const u64 data5_8 = motor5_8.get_packed_data();
			std::memcpy(frame.data.data(), &data5_8, 8);
			speed_controller5_8_pub.publish(frame);
		}

		std::atomic<i16> * get_target_current_p(const unsigned int index)
		{
			if(index < 4) return motor1_4.target_currents + index;
			else return motor5_8.target_currents + (index - 4);
		}
	};
}