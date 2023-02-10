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
#include <variant>

#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include <independent_steering/CanFrame.h>
#include <independent_steering/Linear2D.h>
#include <independent_steering/Angular2D.h>

// #include <crs_lib/uninitialized.hpp> // std::variantでよくね？
#include <robo_master/md.hpp>
#include <robo_master/manager.hpp>
#include <shirasu_md.hpp>
#include <steering_wheel_driver.hpp>

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
					// CRSLib::PidController<float>::Parameters position;
				} steering;

				u32 drive;

				double gear_ratio_steering_par_motor;
				double wheel_radius;
				double distance_from_center;
				double tangent_direction;
			};

			std::variant<std::monostate, ros::NodeHandle> nh;
			double robomasu_pub_tim_duration;
			std::array<Wheel, 4> wheels;
			double steering_limit;

			double deadzone;
		};

		struct Inner final
		{
		private:
			using NormAng = CRSLib::NormalizedAngle::NormalizedAngle<double>;
		
		public:
			struct SteeringWheel final
			{
				crs_lib::SteeringWheelDriver<crs_lib::MotorDriver::ShirasuMd, crs_lib::MotorDriver::RoboMaster::Md> driver;
				const double distance_from_center;
				const double tangent_direction;
			};

			crs_lib::MotorDriver::RoboMaster::RoboMasterMdManager robomasu_manager;
			std::array<std::variant<std::monostate, SteeringWheel>, 4> wheels{};
			const double deadzone;

			ros::Subscriber linear2d_sub{};
			ros::Subscriber angular2d_sub{};
			ros::Timer robomasu_pub_tim{};

			Inner(Config& config):
				robomasu_manager{std::get<1>(config.nh)},
				deadzone{config.deadzone}
			{
				for(unsigned int i = 0; i < 4; ++i)
				{
					wheels[i].emplace<SteeringWheel>
					(
						decltype(SteeringWheel::driver){
							crs_lib::MotorDriver::ShirasuMd{config.wheels[i].steering.id, std::get<1>(config.nh)},
							crs_lib::MotorDriver::RoboMaster::Md{std::get<1>(config.nh), robomasu_manager.get_target_current_p(i), config.wheels[i].steering.current, config.wheels[i].steering.velocity, true},
							config.wheels[i].gear_ratio_steering_par_motor,
							config.wheels[i].wheel_radius,
							config.steering_limit
						},
						config.wheels[i].distance_from_center,
						config.wheels[i].tangent_direction
					);
				}

				linear2d_sub = std::get<1>(config.nh).subscribe<independent_steering::Linear2D>("body_linear_velocity", 1, &Inner::body_linear_velocity_callback, this);
				angular2d_sub = std::get<1>(config.nh).subscribe<independent_steering::Angular2D>("body_angular_velocity", 1, &Inner::body_angular_velocity_callback, this);
				robomasu_pub_tim = std::get<1>(config.nh).createTimer(ros::Duration{config.robomasu_pub_tim_duration}, &Inner::robomasu_pub_tim_callback, this);
			}

			// Crab制御
			void body_linear_velocity_callback(const independent_steering::Linear2D::ConstPtr& body_linear_velocity)
			{
				const double norm = std::sqrt(body_linear_velocity->x * body_linear_velocity->x + body_linear_velocity->y * body_linear_velocity->y);
				if(norm < deadzone)
				{
					for(auto& wheel : wheels)
					{
						std::get<1>(wheel).driver.update(NormAng{0.0}, 0.0);
					}
				}
				else
				{
					const double theta = std::atan2(body_linear_velocity->y, body_linear_velocity->x);

					for(auto& wheel : wheels)
					{
						std::get<1>(wheel).driver.update(NormAng{theta}, norm);
					}
				}
			}

			// Spinning制御
			void body_angular_velocity_callback(const independent_steering::Angular2D::ConstPtr& body_angular_velocity)
			{
				for(auto& wheel : wheels)
				{
					auto&& wheel_inner = std::get<1>(wheel);
					wheel_inner.driver.update(NormAng{wheel_inner.tangent_direction}, body_angular_velocity->z * wheel_inner.distance_from_center);
				}
			}

			void robomasu_pub_tim_callback(const ros::TimerEvent&)
			{
				robomasu_manager.publish();
			}
		};

		// onInitまで未初期化であるようにするため。
		std::variant<std::monostate, Inner> inner;

		void onInit() override
		{
			auto nh = getNodeHandle();
			
			Config config;
			config.nh = nh;
			/// TODO: 各コンフィグの読み込み(rosparam_util)

			inner.emplace<Inner>(config);
		}
	};
}