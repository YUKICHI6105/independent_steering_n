#pragma once

#include <cmath>
#include <numbers>
#include <utility>

#include "gearbox.hpp"

namespace nhk2024::independent_steering_n::steering_wheel
{
	struct SteeringWheel final
	{
		double current_driving_speed;

		static auto make() noexcept
		{
			return SteeringWheel{0.0};
		}

		auto inverse(double steering_angle, const double driving_speed) noexcept -> std::pair<double, double>
		{
			const bool reverse_angle = std::signbit(current_driving_speed) ^ std::signbit(driving_speed);
			current_driving_speed = driving_speed;

			if(reverse_angle)
			{
				return {steering_angle + std::numbers::pi, driving_speed};
			}
			else
			{
				return {steering_angle, driving_speed};
			}
		}

		void clear() noexcept
		{
			current_driving_speed = 0.0;
		}
	};
}