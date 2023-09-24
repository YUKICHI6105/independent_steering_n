#pragma once

#include "rational.hpp"

namespace nhk2024::independent_steering_n::gearbox
{
	template<class T>
	concept motor = requires(const T imut)
	{
		{imut.send_target(0.0f)} noexcept;
	};

	struct Gearedbox final
	{
		rational::Rational shaft_per_motor;

		auto inverse(const double target) const noexcept -> double
		{
			return shaft_per_motor.inverse().template to<double>() * target;
		}
	};
}