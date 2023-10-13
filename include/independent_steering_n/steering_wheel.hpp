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
		double current_angle;

		static auto make() noexcept
		{
			return SteeringWheel{0.0};
		}

		auto inverse(const double steering_angle, const double driving_speed) noexcept -> std::pair<double, double>
		{
			// 差を2πで割った余りを求める
			const double diff = std::fmod(steering_angle - current_angle, std::numbers::pi * 2.0);
			current_angle += diff;

			// 差がπより大きい場合は、操舵をdiff ± 2π回転させ(すなわち逆向きに回し)、駆動の符号を反転させる
			if(std::abs(diff) > std::numbers::pi)
			{
				current_angle += (diff > 0.0 ? -std::numbers::pi * 2.0 : std::numbers::pi * 2.0);
				return std::make_pair(current_angle, -driving_speed);
			}
			{
				return std::make_pair(current_angle, driving_speed);
			}
		}

		auto stop() noexcept -> double
		{
			return current_angle;
		}
	};
}