#pragma once

#include <ros/ros.h>

#include <CanFrame.h>

#include "speed_controller_message.hpp"
#include "md.hpp"

namespace crs_lib::MotorDriver::RoboMaster
{
	class RoboMasterMdManager final
	{
		ros::Publisher speed_controller1_4_pub;
		ros::Publisher speed_controller5_8_pub;

	public:
		SpeedControllerMessage motor1_4{};
		SpeedControllerMessage motor5_8{};

		RoboMasterMdManager(ros::NodeHandle& nh, adhoc_can_plugins2::CallbackManager& callback_manager):
			motor1_4{nh.advertise<independent_steering::CanFrame>("can" + std::to_string(0x200), 1)},
			motor5_8{nh.advertise<independent_steering::CanFrame>("can" + std::to_string(0x1FF), 1)}
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
	};
}