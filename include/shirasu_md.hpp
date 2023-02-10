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
#include <atomic>

#include <ros/ros.h>

#include <independent_steering/CanFrame.h>

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
		std::atomic<bool> is_ajusting_zero{false};
		std::atomic<bool> is_entered_homing{false};
		std::atomic<float> current_target_velocity{std::nan("")};
		std::atomic<float> current_target_position{std::nan("")};
		
		ros::Publisher can_mode_pub;
		ros::Publisher can_target_pub;
		ros::Subscriber can_status_sub;

	public:
		ShirasuMd(const u32 base_id, ros::NodeHandle& nh):
			base_id{base_id},
			can_mode_pub{nh.advertise<independent_steering::CanFrame>("can/" + std::to_string(base_id), 10)},
			can_target_pub{nh.advertise<independent_steering::CanFrame>("can/" + std::to_string(base_id + 1), 1)},
			can_status_sub{nh.subscribe<independent_steering::CanFrame>("can/" + std::to_string(base_id + 3), 1, &ShirasuMd::update_status, this)}
		{}

		ShirasuMd(ShirasuMd&& md):
			base_id{md.base_id},
			can_mode_pub{std::move(md.can_mode_pub)},
			can_target_pub{std::move(can_target_pub)},
			can_status_sub{std::move(can_status_sub)}
		{}

		void velocity_update(const float target)
		// [[expects: mode == ShirasuMode::velocity]]
		{
			if(mode != ShirasuMode::velocity)
			{
				/// @todo error report
				return;
			}
			current_target_velocity = target;
			update(target);
		}

		float get_velocity()
		{
			return current_target_velocity;
		}

		void position_update(const float target)
		// [[expects: mode == ShirasuMode::position]]
		{
			if(mode != ShirasuMode::position)
			{
				/// @todo error report
				return;
			}
			current_target_position = target;
			update(target);
		}

		float get_position()
		{
			return current_target_position;
		}

		void ajust_zero_point()
		{
			is_ajusting_zero = true;
			change_mode(ShirasuMode::homing);
		}

		bool finished_ajusting_zero()
		{
			return !is_ajusting_zero;
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
			default:
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
			independent_steering::CanFrame frame{};
			frame.dlc = sizeof(ShirasuMode);
			frame.data[0] = CRSLib::to_underlying(mode);

			{
				std::lock_guard lock{mutex};
				can_mode_pub.publish(frame);
			}
		}
	
	private:
		void update(const float target)
		{
			independent_steering::CanFrame frame{};
			frame.dlc = sizeof(float);
			adhoc_can_plugins2::pack<std::endian::big>(frame.data.data(), target);
			can_target_pub.publish(frame);
		}

		void update_status(const independent_steering::CanFrame::ConstPtr& frame)
		{
			std::lock_guard lock{mutex};

			mode = static_cast<ShirasuMode>(frame->data[0]);
			if(is_ajusting_zero)
			{
				if(mode == ShirasuMode::homing)
				{
					is_entered_homing = true;
				}
				else if(is_entered_homing)
				{
					is_ajusting_zero = false;
					is_entered_homing = false;
				}
			}
		}
	};

	static_assert(MotorDriver<ShirasuMd>);
	static_assert(VelocityControlable<ShirasuMd>);
	static_assert(PositionControlable<ShirasuMd>);
}