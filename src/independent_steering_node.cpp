/**
 * @file independent_steering_node.cpp
 * @author Stew
 * @brief N輪独立ステアリングを制御する。
 * spinning制御(回転運動のみ)とcrab制御(並進運動のみ)の2モードを扱う。
 * @version 0.1
 * @date 2022-11-17
 * 
 * @todo RosparamUtilの改善。型と変数名の組を使って、一次オブジェクトを作り、そしてmemcpyとかなら上手くいくと思った。
 */

#include <utility>
#include <array>
#include <optional>

#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include <independent_steering/CanFrame.h>
#include <independent_steering/Linear2D.h>
#include <independent_steering/Angular2D.h>

// #include <crs_lib/uninitialized.hpp> // std::optionalでよくね？
#include <crs_lib/rosparam_util.hpp>
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

				double gear_ratio_steering_par_motor;
				double wheel_radius;
				double distance_from_center;
				double tangent_direction;
			};

			std::optional<ros::NodeHandle> nh;
			double robomasu_pub_tim_duration;
			double steering_limit;
			double deadzone;
			std::array<Wheel, 4> wheels;

			Config(ros::NodeHandle& nh)
			{
				using CRSLib::RosparamUtil::get_param;
				using CRSLib::RosparamUtil::read_param;
				using CRSLib::RosparamUtil::Param;

				this->nh = nh;
				auto independent_steering_node_param = get_param(nh, "independent_steering_node");
				robomasu_pub_tim_duration = read_param<double>(independent_steering_node_param, "pub_tim_duration");
				steering_limit = read_param<double>(independent_steering_node_param, "steering_limit");
				deadzone = read_param<double>(independent_steering_node_param, "deadzone");

				auto wheels_param = get_param(independent_steering_node_param, "wheels");
				for(int i = 0; i < 4; ++i)
				{
					auto wheel_param = get_param(wheels_param, i);

					{
						auto steering_param = get_param(wheel_param, "steering");

						wheels[i].steering.id = read_param<int>(steering_param, "id");
						
						constexpr auto init_pid_param = [](CRSLib::PidController<float>::Parameters& pid_param, const Param& pid_rosparam)
						{
							pid_param =
							{
								static_cast<float>(read_param<double>(pid_rosparam, "p")),
								static_cast<float>(read_param<double>(pid_rosparam, "i")),
								static_cast<float>(read_param<double>(pid_rosparam, "d")),
							};
						};
						
						init_pid_param(wheels[i].steering.current, get_param(steering_param, "current"));
						init_pid_param(wheels[i].steering.velocity, get_param(steering_param, "velocity"));
					}

					wheels[i].gear_ratio_steering_par_motor = read_param<double>(wheel_param, "gear_ratio_steering_per_motor");
					wheels[i].wheel_radius = read_param<double>(wheel_param, "wheel_radius");
					wheels[i].distance_from_center = read_param<double>(wheel_param, "distance_from_center");
					wheels[i].tangent_direction = read_param<double>(wheel_param, "tangent_direction");
				}
			}
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
			std::array<std::optional<SteeringWheel>, 4> wheels{};
			const double deadzone;

			ros::Subscriber linear2d_sub{};
			ros::Subscriber angular2d_sub{};
			ros::Timer robomasu_pub_tim{};

			Inner(Config& config):
				robomasu_manager{*config.nh},
				deadzone{config.deadzone}
			{
				for(unsigned int i = 0; i < 4; ++i)
				{
					wheels[i].emplace
					(
						decltype(SteeringWheel::driver){
							crs_lib::MotorDriver::ShirasuMd{config.wheels[i].steering.id, *config.nh},
							crs_lib::MotorDriver::RoboMaster::Md{*config.nh, robomasu_manager.get_target_current_p(i), config.wheels[i].steering.current, config.wheels[i].steering.velocity, true},
							config.wheels[i].gear_ratio_steering_par_motor,
							config.wheels[i].wheel_radius,
							config.steering_limit
						},
						config.wheels[i].distance_from_center,
						config.wheels[i].tangent_direction
					);
				}

				linear2d_sub = (*config.nh).subscribe<independent_steering::Linear2D>("body_linear_velocity", 1, &Inner::body_linear_velocity_callback, this);
				angular2d_sub = (*config.nh).subscribe<independent_steering::Angular2D>("body_angular_velocity", 1, &Inner::body_angular_velocity_callback, this);
				robomasu_pub_tim = (*config.nh).createTimer(ros::Duration{config.robomasu_pub_tim_duration}, &Inner::robomasu_pub_tim_callback, this);
			}

			// Crab制御
			void body_linear_velocity_callback(const independent_steering::Linear2D::ConstPtr& body_linear_velocity)
			{
				const double norm = std::sqrt(body_linear_velocity->x * body_linear_velocity->x + body_linear_velocity->y * body_linear_velocity->y);
				if(norm < deadzone)
				{
					for(auto& wheel : wheels)
					{
						(*wheel).driver.update(NormAng{0.0}, 0.0);
					}
				}
				else
				{
					const double theta = std::atan2(body_linear_velocity->y, body_linear_velocity->x);

					for(auto& wheel : wheels)
					{
						(*wheel).driver.update(NormAng{theta}, norm);
					}
				}
			}

			// Spinning制御
			void body_angular_velocity_callback(const independent_steering::Angular2D::ConstPtr& body_angular_velocity)
			{
				for(auto& wheel : wheels)
				{
					auto&& wheel_inner = *wheel;
					wheel_inner.driver.update(NormAng{wheel_inner.tangent_direction}, body_angular_velocity->z * wheel_inner.distance_from_center);
				}
			}

			void robomasu_pub_tim_callback(const ros::TimerEvent&)
			{
				robomasu_manager.publish();
			}
		};

		// onInitまで未初期化であるようにするため。
		std::optional<Inner> inner;

		void onInit() override
		{
			Config config{getNodeHandle()};

			inner.emplace(config);
		}
	};
}