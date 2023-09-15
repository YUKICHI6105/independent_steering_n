#pragma once
#include <cstdint>
#include <optional>

#include <std_msgs/msg/u_int8.hpp>

#include "utility.hpp"

namespace nhk2024::independent_steering_n::control_mode
{
	enum class ControlMode : std::uint8_t
	{
		disable,
		crab,
		spinning
	};

	inline auto to_control_mode(const std_msgs::msg::UInt8& msg) noexcept -> std::optional<ControlMode>
	{
		switch(msg.data)
		{
			case 0:
				return ControlMode::disable;
			case 1:
				return ControlMode::crab;
			case 2:
				return ControlMode::spinning;
			default:
				return std::nullopt;
		}
	}

	inline auto control_mode_msg(const ControlMode mode) noexcept -> std_msgs::msg::UInt8
	{
		auto msg = std_msgs::msg::UInt8{};
		msg.data = utility::to_underlying(mode);
		return msg;
	}
}