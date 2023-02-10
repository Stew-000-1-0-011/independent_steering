#pragma once

#include <cmath>
#include <numbers>

namespace CRSLib::NormalizedAngle
{
	// [-pi, pi]であることを型名で保証(このクラス内では検査しない)
	template<class T>
	struct NormalizedAngle final
	{
		T value{};

		explicit constexpr NormalizedAngle(const T& value) noexcept:
			value{value}
		{}

		constexpr operator const T&() const& noexcept
		{
			return value;
		}

		constexpr operator T&() & noexcept
		{
			return value;
		}

		constexpr operator const T&&() const&& noexcept
		{
			return std::move(value);
		}

		constexpr operator T&&() && noexcept
		{
			return std::move(value);
		}

		constexpr NormalizedAngle reverse() const noexcept
		{
			if(value > 0) return std::numbers::pi_v<T> - value;
			else return std::numbers::pi_v<T> + value;
		}
	};

	template<class T>
	inline constexpr auto normalize_angle(const T& angle) noexcept -> NormalizedAngle<T>
	{
		T current_angle = std::fmod(angle, 2 * std::numbers::pi_v<T>);

		if(current_angle < -std::numbers::pi_v<T>) current_angle += 2 * std::numbers::pi_v<T>;
		else if(std::numbers::pi_v<T> < current_angle) current_angle -= 2 * std::numbers::pi_v<T>;

		return NormalizedAngle<T>{current_angle};
	}

	template<class T, class U>
	inline constexpr auto minimum_diff(const NormalizedAngle<T> x, const NormalizedAngle<U> y) noexcept
	{
		return normalize_angle(x - y);
	}
}