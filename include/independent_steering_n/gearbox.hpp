#pragma once

#include <utility>
#include <concepts>
#include "shirasu.hpp"
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
		rational::Rational from_motor_to_shaft;

		auto inverse(const double target) const noexcept -> double
		{
			return from_motor_to_shaft.template to<double>() * target;
		}
	};
}