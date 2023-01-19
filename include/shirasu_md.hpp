#pragma once

/**
 * @file shirasu_md.hpp
 * @author Stew
 * @brief シラスのモタドラ。
 * メンバ関数の呼び出しは互いにスレッドセーフである。
 * @version 0.1
 * @date 2022-11-23
 */

#include <shared_mutex>
#include <string>

#include <ros/ros.h>

#include <can_plugins/include/Frame.h>

#include <CRSLib/include/utility.hpp>

#include "std_type.hpp"
#include "motor_driver.hpp"
#include "adhoc_can_plugins2_pack.hpp"

namespace crs_lib::MotorDriver
{
	/// @brief シラスのモータードライバ。
	class ShirasuMd final
	{
		u32 base_id;

	public:
		enum class ShirasuMode : u8
		{
			disable = 0,
			default_ = 1,
			homing = 2,
			reserved,
			current = 4,
			velocity = 5,
			position = 6
		};

	private:
		ShirasuMode mode{ShirasuMode::disable};
		mutable std::shared_mutex mutex{};
		
		ros::Publisher can_mode_pub;
		ros::Publisher can_target_pub;

	public:
		ShirasuMd(const u32 base_id, ros::NodeHandle& nh):
			base_id{base_id},
			can_mode_pub{nh.advertise<can_plugins::Frame>("can/" + std::to_string(base_id), 10)},
			can_target_pub{nh.advertise<can_plugins::Frame>("can/" + std::to_string(base_id + 1), 1)}
		{}

		void velocity_update(const float target)
		// [[expects: mode == ShirasuMode::velocity]]
		{
			update(target);
		}

		void position_update(const float target)
		// [[expects: mode == ShirasuMode::position]]
		{
			update(target);
		}

		MotorDriver::Mode get_mode() const noexcept
		{
			std::shared_lock lock{mutex};
			switch(mode)
			{
			case ShirasuMode::disable:
				return MotorDriver::Mode::stop;

			case ShirasuMode::homing:
				return MotorDriver::Mode::powerless;

			case ShirasuMode::current:
			case ShirasuMode::velocity:
			case ShirasuMode::position:
				return MotorDriver::Mode::run;

			// default_, reservedにするな。さもなくばエラー。
			case default:
				return MotorDriver::Mode::error;
			}
		}

		void to_stop()
		{
			change_mode(ShirasuMode::disable);
		}

		[[deprecated]] void to_powerless()
		{
			/// @todo 実装
		}

		/**
		 * @brief シラスモードを変更する。
		 * 
		 * @param mode ShirasuMode。チェックはしていないがdefault_とreservedには変更しないこと。
		 */
		void change_mode(const ShirasuMode mode)
		{
			can_plugins::Frame frame{};
			frame.id = base_id;
			frame.dlc = sizeof(ShirasuMode);
			frame.data[0] = to_underlying(mode);

			{
				std::lock_guard lock{mutex};
				can_mode_pub.publish(frame);
				this.mode = mode;
			}
		}
	
	private:
		void update(const float target)
		{
			can_plugins::Frame frame{};
			frame.id = base_id + 1;
			frame.dlc = sizeof(float);
			can_plugins2::pack(frame.data.data(), target);
			can_target_pub.publish(frame);
		}
	};

	static_assert(MotorDriverC<ShirasuMd>);
	static_assert(VelocityControlableC<ShirasuMd>);
	static_assert(PositionControlableC<ShirasuMd>);
}