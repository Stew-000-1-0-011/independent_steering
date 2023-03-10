#pragma once

#include <concepts>

namespace crs_lib::MotorDriver
{
	enum class Mode
	{
		run,
		stop,
		powerless,

		error
	};

	template<class T>
	concept MotorDriver = requires(T motor_driver, const T const_motor_driver, const float target)
	{
		requires std::move_constructible<T>;
		{const_motor_driver.get_mode()} noexcept -> std::same_as<Mode>;
		motor_driver.to_stop();
		motor_driver.to_powerless();
	};

	template<class T>
	concept VelocityControlable = MotorDriver<T> && requires(T motor_driver, const float target)
	{
		motor_driver.velocity_update(target);
		{motor_driver.get_velocity()} -> std::same_as<float>;
	};

	template<class T>
	concept PositionControlable = MotorDriver<T> && requires(T motor_driver, const float target)
	{
		motor_driver.position_update(target);
		{motor_driver.get_position()} -> std::same_as<float>;
		{motor_driver.ajust_zero_point()};
		{motor_driver.finished_ajusting_zero()} -> std::same_as<bool>;
	};
}