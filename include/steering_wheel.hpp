/**
 * @file steering_wheel.hpp
 * @author Stew
 * @brief 1つのステアリングホイール。
 * 目標速度・目標位置にどれほど追従できているか、その目標値が不正か(適正範囲か、変化量は大きすぎないか)は考慮しない。
 * このクラスを用いる側がそこらへんは考慮すること
 * @version 0.1
 * @date 2022-11-23
 * 
 */

#include <utility>

namespace crs_lib
{
	/**
	 * @tparam SteeringMd ステアリング角を位置制御するモータードライバ
	 * @tparam DrivingMd 駆動輪を速度制御するモータードライバ
	 */
	template<class SteeringMd, class DrivingMd>
	class SteeringWheel final
	{
		SteeringMd steering_md;
		DrivingMd driving_md;
		const double gear_ratio_steering_par_motor;
		const double wheel_radius_reciprocal;

	public:
		/**
		 * @brief Construct a new Steering Wheel object
		 * 
		 * @param steering_md 操舵用モーターのモタドラ
		 * @param driving_md 駆動用モーターのモタドラ
		 * @param gear_ratio_steering_par_motor 変速比(ステアリング角 / 目標位置角)
		 * @param wheel_radius ホイール半径
		 */
		SteeringWheel(SteeringMd&& steering_md, DrivingMd&& driving_md, const double gear_ratio_steering_par_motor, const double wheel_radius):
			steering_md{std::move(steering_md)},
			driving_md{std::move(driving_md)},
			gear_ratio_steering_par_motor{gear_ratio_steering_par_motor},
			wheel_radius_reciprocal{1.0 / wheel_radius}
		{}

		/**
		 * @brief 目標値を更新する。
		 * 
		 * @param steering_angle ホイールのステアリング角
		 * @param driving_velocity ホイールの駆動部の角速度 * ホイール半径
		 */
		void update(const double steering_angle, const double driving_velocity)
		{
			steering_md.update(steering_angle * gear_ratio_steering_par_motor);
			driving_velocity.update(driving_velocity * wheel_radius_reciprocal);
		}
	};
}