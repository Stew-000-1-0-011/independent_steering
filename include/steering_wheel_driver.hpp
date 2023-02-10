/**
 * @file steering_wheel_driver.hpp
 * @author Stew
 * @brief 1つのステアリングホイール。
 * 目標速度・目標位置にどれほど追従できているか、その目標値が不正か(適正範囲か、変化量は大きすぎないか)は考慮しない。
 * このクラスを用いる側がそこらへんは考慮すること。
 * また、位置を表す変数がdouble型でオーバーフローしないだろうと仮定している。起動し続けるようなロボットには使わないこと。
 * @version 0.1
 * @date 2022-11-23
 */

#include <utility>
#include <cmath>
#include <numbers>

#include "motor_driver.hpp"
#include "utility.hpp"
#include "normalized_angle.hpp"

namespace crs_lib
{
	/**
	 * @tparam SteeringMd ステアリング角を位置制御するモータードライバ
	 * @tparam DrivingMd 駆動輪を速度制御するモータードライバ
	 */
	template<crs_lib::MotorDriver::PositionControlable SteeringMd, crs_lib::MotorDriver::VelocityControlable DrivingMd>
	class SteeringWheelDriver final
	{
		SteeringMd steering_md;
		DrivingMd driving_md;
		const double gear_ratio_steering_par_motor;
		const double gear_ratio_motor_par_steering;
		const double wheel_radius_reciprocal;
		const double motor_limit;

	public:
		/**
		 * @brief Construct a new Steering Wheel object
		 * 
		 * @param steering_md 操舵用モーターのモタドラ
		 * @param driving_md 駆動用モーターのモタドラ
		 * @param gear_ratio_steering_par_motor 変速比(ステアリング角 / モーターの目標位置角)
		 * @param wheel_radius ホイール半径
		 * @param steer_limit ステアリング可能な角度の絶対値
		 */
		SteeringWheelDriver(SteeringMd&& steering_md, DrivingMd&& driving_md, const double gear_ratio_steering_par_motor, const double wheel_radius, const double steer_limit):
			steering_md{std::move(steering_md)},
			driving_md{std::move(driving_md)},
			gear_ratio_steering_par_motor{gear_ratio_steering_par_motor},
			gear_ratio_motor_par_steering{1.0 / gear_ratio_steering_par_motor},
			wheel_radius_reciprocal{1.0 / wheel_radius},
			motor_limit{steer_limit * gear_ratio_motor_par_steering}
		{}

		/**
		 * @brief 目標値を更新する。
		 * 
		 * @param steering_angle ホイールのステアリング角。
		 * @param driving_velocity ホイールの駆動部の角速度 * ホイール半径
		 */
		void update(const CRSLib::NormalizedAngle::NormalizedAngle<double> steering_angle, const double driving_velocity)
		{
			using CRSLib::NormalizedAngle::normalize_angle;
			using CRSLib::NormalizedAngle::minimum_diff;

			const auto current_motor = steering_md.get_position();
			const auto current_steer = normalize_angle(current_motor * gear_ratio_steering_par_motor);
			const auto target_diff = minimum_diff(steering_angle, current_steer);
			const double target = current_motor + target_diff * gear_ratio_motor_par_steering;

			if(std::abs(target) > motor_limit)
			{
				/// @todo Warning Report
				return;
			}

			steering_md.position_update(target);
			driving_md.velocity_update(driving_velocity * wheel_radius_reciprocal);
		}

		/**
		 * @brief 零点合わせを行う
		*/
		void ajust_zero_point()
		{
			steering_md.ajust_zero_point();
		}

		/**
		 * @brief 零点合わせの完了を知る
		 * 
		 * @return true 零点合わせしていない
		 * @return false 零点合わせ中
		 */
		bool finished_ajusting_zero()
		{
			return steering_md.finished_ajusting_zero();
		}
	};
}