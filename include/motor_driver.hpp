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
	concept MotorDriverC = requires(T motor_driver, const T const_motor_driver, const float target)
	{
		requires std::move_constructible<T>;
		{const_motor_driver.get_mode()} noexcept -> std::same_as<Mode>;
		motor_driver.to_stop();
		motor_driver.to_powerless();
	};

	template<class T>
	concept VelocityControlableC = MotorDriverC<T> && requires(T motor_driver, const float target)
	{
		motor_driver.velocity_update(target);
	};

	template<class T>
	concept PositionControlableC = MotorDriverC<T> && requires(T motor_driver, const float target)
	{
		motor_driver.position_update(target);
	};
}