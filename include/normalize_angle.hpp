#pragma once

#include <cmath>
#include <numbers>

namespace crs_lib
{
	constexpr double normalize_angle(const double angle) noexcept
	{
		double current_angle = std::fmod(angle, 2 * std::numbers::pi);
		
		if(current_angle < -std::numbers::pi) current_angle += 2 * std::numbers::pi;
		else if(std::numbers::pi < current_angle) current_angle -= 2 * std::numbers::pi;

		return current_angle;
	}
}