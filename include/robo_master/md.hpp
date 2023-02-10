#pragma once

#include <cstring>
#include <shared_mutex>
#include <string>
#include <utility>
#include <atomic>

#include <ros/ros.h>

#include <CRSLib/include/pid_controller.hpp>

#include <independent_steering/CanFrame.h>

#include "../std_type.hpp"
// #include "../adhoc_can_callback_manager.hpp"
#include "../motor_driver.hpp"

namespace crs_lib::MotorDriver::RoboMaster
{
	class Md final
	{
		friend class Manager;
	public:
		enum class Mode
		{
			velocity,
			position,
			current,
			stop,
			powerless,

			error
		};

		struct CurrentState final
		{
			u16 position{};
			i16 velocity{};
			i16 current{};
			u8 temperature{};
		};

	private:
		mutable std::shared_mutex current_state_mutex{};
		mutable std::shared_mutex mode_mutex{};

		CRSLib::PidController<float> current_pid;
		CRSLib::PidController<float> velocity_pid;
		// CRSLib::PidController<float> position_pid;

		CurrentState current_state{};

		std::atomic<i16> * target_current_p;

		Mode mode{Mode::stop};
		
		ros::Subscriber can_sub;

	public:
		Md(ros::NodeHandle& nh, std::atomic<i16> *const target_current_p, CRSLib::PidController<float>::Parameters& current, CRSLib::PidController<float>::Parameters& velocity, /*CRSLib::PidController<float>::Parameters& position,*/ const bool is_1_4/*, adhoc_can_plugins2::CallbackManager& callback_manager*/):
			current_pid{current},
			velocity_pid{velocity},
			// position_pid{position},
			target_current_p{target_current_p},
			can_sub{nh.subscribe<independent_steering::CanFrame>("can/" + is_1_4 ? std::to_string(0x200) : std::to_string(0x1FF), 1, &Md::robo_master_can_callback, this)}
		{
			// なんだこれ...
			// callback_manager.push();
		}

		Md(Md&& other):
			current_pid{std::move(other.current_pid)},
			velocity_pid{std::move(other.velocity_pid)},
			// position_pid{std::move(other.position_pid)},
			current_state{other.current_state},
			target_current_p{other.target_current_p},
			mode{other.mode},
			can_sub{std::move(other.can_sub)}
		{}

	public:
		MotorDriver::Mode get_mode() const noexcept
		{
			std::shared_lock lock{mode_mutex};

			switch(mode)
			{
			case Mode::velocity:
			case Mode::position:
			case Mode::current:
				return MotorDriver::Mode::run;
			
			case Mode::stop:
				return MotorDriver::Mode::stop;

			case Mode::powerless:
				return MotorDriver::Mode::powerless;

			case Mode::error:
				return MotorDriver::Mode::error;
			}
		}

		void to_stop()
		{
			velocity_update(0.0f);
		}

		void to_powerless()
		{
			current_update(0.0f);
		}

		void velocity_update(const float target)
		{
			velocity_pid.update(target);
			const float control_input = velocity_pid.calculate(target);

			current_update(target);
		}

		// void position_update(const float target)
		// {
		// 	position_pid.update(target);
		// 	const float control_input = position_pid.calculate(target);

		// 	velocity_update(target);
		// }

		/// @brief update current.
		/// @param target Arbitrary units
		void current_update(const float target)
		{
			current_pid.update(target);
			const float control_input = current_pid.calculate(target);

			if(control_input < -16384.0) *target_current_p = -16384;
			else if(16384 < control_input) *target_current_p = 16384;
			else *target_current_p = static_cast<i16>(control_input);
		}

		CurrentState get_current_state() const
		{
			std::shared_lock lock{current_state_mutex};
			return current_state;
		}

		float get_velocity() const
		{
			return get_current_state().velocity;
		}

	private:
		void robo_master_can_callback(const independent_steering::CanFrame::ConstPtr& frame) noexcept
		{
			std::lock_guard lock{current_state_mutex};
			std::memcpy(&current_state, frame->data.data(), 7);
		}
	};

	static_assert(MotorDriver<Md>);
	static_assert(VelocityControlable<Md>);
	// static_assert(PositionControlableC<Md>);
}