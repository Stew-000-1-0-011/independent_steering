#pragma once

#include <cstring>
#include <shared_mutex>
#include <string>
#include <utility>
#include <atomic>

#include <ros/ros.h>

#include <CRSLib/include/pid_controller.hpp>

#include <CanFrame.h>

#include "../std_type.hpp"
#include "../adhoc_can_callback_manager.hpp"
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
		CRSLib::PidController<float> position_pid;

		CurrentState current_state{};

		std::atomic<i16> * target_current_p;

		Mode mode{Mode::stop};
		int index;

	public:
		RoboMasterMd(const int index, std::atomic<i16> *const target_current_p, CRSLib::PidController<float>&& current_pid, CRSLib::PidController<float>&& velocity_pid, CRSLib::PidController<float>&& position_pid, adhoc_can_plugins2::CallbackManager& callback_manager):
			current_pid{std::move(current_pid)},
			velocity_pid{std::move(velocity_pid)},
			position_pid{std::move(position_pid)},
			target_current_p{target_current_p},
			index{index}
		{
			callback_manager.push();
		}

		RoboMasterMd(RoboMasterMd&& other):
			current_pid{std::move(other.current_pid)},
			velocity_pid{std::move(other.velocity_pid)},
			position_pid{std::move(other.position_pid)},
			current_state{other.current_state},
			mode{other.mode},
			index{other.index}
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

		void position_update(const float target)
		{
			position_pid.update(target);
			const float control_input = position_pid.calculate(target);

			velocity_update(target);
		}

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

	private:
		void robo_master_can_callback(const char *const data) noexcept
		{
			std::lock_guard lock{current_state_mutex};
			std::memcpy(data, &current_state, 7);
		}
	};

	static_assert(MotorDriverC<RoboMasterMd>);
	static_assert(VelocityControlableC<RoboMasterMd>);
	static_assert(PositionControlableC<RoboMasterMd>);
}