#pragma once

#include <cmath>
#include <numbers>
#include <utility>

#include "gearbox.hpp"

namespace nhk2024::independent_steering_n::steering_wheel
{
	constexpr double enough_slow = 0.001;
	struct SteeringWheel final
	{
		double driving_speed_average;
		double current_angle;

		static auto make() noexcept
		{
			return SteeringWheel{0.0, 0.0};
		}

		auto inverse(double steering_angle, const double driving_speed) noexcept -> std::pair<double, double>
		{
			
			driving_speed_average = 0.9 * driving_speed_average + 0.1 * driving_speed;
			const bool reverse_angle = std::signbit(driving_speed_average) ^ std::signbit(driving_speed) && std::abs(driving_speed_average) > enough_slow;
			if(reverse_angle)
			{
				if(steering_angle > 0.0)
				{
					steering_angle -= std::numbers::pi;
				}
				else
				{
					steering_angle += std::numbers::pi;
				}
			}

			current_angle = steering_angle;
			return {steering_angle, driving_speed};
		}

		auto stop() noexcept -> double
		{
			driving_speed_average = 0.0;
			return current_angle;
		}

		void clear() noexcept
		{
			driving_speed_average = 0.0;
		}
	};
}