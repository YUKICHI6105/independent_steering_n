#pragma once

#include <cstdint>
#include <concepts>

namespace nhk2024::independent_steering_n::rational
{
	struct Rational final
	{
		std::uint32_t numerator;
		std::uint32_t denominator{1};

		friend auto operator*(const Rational& lhs, const Rational& rhs) noexcept -> Rational
		{
			return Rational{lhs.numerator * rhs.numerator, lhs.denominator * rhs.denominator};
		}

		template<std::floating_point F>
		auto to() const noexcept -> F
		{
			return static_cast<F>(numerator) / static_cast<F>(denominator);
		}

		auto inverse() const noexcept -> Rational
		{
			return Rational{denominator, numerator};
		}
	};
}