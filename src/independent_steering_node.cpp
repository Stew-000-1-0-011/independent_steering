/**
 * @file independent_steering_node.cpp
 * @author Stew
 * @brief N輪独立ステアリングを制御する。
 * spinning制御(回転運動のみ)とcrab制御(並進運動のみ)の2モードを扱う。
 * @version 0.1
 * @date 2022-11-17
 * 
 */

#include <utility>
#include <array>

#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <geometry_msgs/Twist.h>

#include <independent_steering/CanFrame.h>

#include <crs_lib/uninitialized.hpp>
#include <robo_master/md.hpp>
#include <robo_master/manager.hpp>
#include <shirasu_md.hpp>
#include <steering_wheel.hpp>

namespace crs_lib
{
	class IndependentSteeringNode final : public nodelet::Nodelet
	{
		struct Config final
		{
			struct Wheel
			{
				struct Steering
				{
					u32 id;
					CRSLib::PidController<float>::Parameters current;
					CRSLib::PidController<float>::Parameters velocity;
					CRSLib::PidController<float>::Parameters current;
				} steering;

				u32 drive;

				double gear_ratio_steering_par_motor;
				double wheel_radius;
			};

			ros::NodeHandle nh;
			double robomasu_pub_tim_duration;
			std::array<Wheel, 4> wheels;
		};

		enum class Mode
		{
			Crab,
			Spinning
		};

		struct Inner final
		{
			crs_lib::MotorDriver::RoboMaster::RoboMasterMdManager robomasu_manager;
			std::array<crs_lib::Uninitialized<crs_lib::SteeringWheel<crs_lib::MotorDriver::ShirasuMd, crs_lib::MotorDriver::RoboMaster::RoboMasterMdManager>>, 4> wheels{};

			ros::Subscriber body_twist_sub{};
			ros::Timer robomasu_pub_tim{};

			Inner(Config&& config):
				robomasu_manager{config.nh}
			{
				for(unsigned int i = 0; i < 4; ++i)
				{
					wheels[i].v =
					{
						crs_lib::MotorDriver::ShirasuMd{config.wheels[i].steering.id, config.nh},
						crs_lib::MotorDriver::RoboMaster::Md{robomasu_manager.motor1_4.get_target_current_p(i), config.wheels[i].steering.current, config.wheels[i].steering.velocity, config.wheels[i].steering.position},
						config.wheels[i].gear_ratio_steering_par_motor,
						config.wheels[i].wheel_radius
					};
				}

				body_twist_sub = config.nh.subscribe<geometry_msgs::Twist>("body_twist", &Inner::body_twist_callback);
				robomasu_pub_tim = config.nh.createTimer(ros::Duration{config.robomasu_pub_tim_duration}, &Inner::robomasu_pub_tim_callback);
			}

			void body_twist_callback(const geometry_msgs::Twist::ConstPtr& body_twist)
			{
				/// TODO: 独立ステアリング制御
			}

			void robomasu_pub_tim_callback(const ros::TimerEvent&)
			{
				robomasu_manager.publish();
			}
		};

		// onInitまで未初期化であるようにするため。
		crs_lib::Uninitialized<Inner> inner;

		void onInit() override
		{
			auto nh = getNodeHandle();
			
			Config config;
			config.nh = nh;
			/// TODO: 各コンフィグの読み込み(rosparam_util)

			inner.v = {config};
		}
	};
}