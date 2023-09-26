#pragma once

#include <cstdint>
#include "not_canplugins2_part.hpp"
namespace nhk2024::independent_steering_n::robomas_utils
{
	inline std::unique_ptr<robo_messages::RobomasFrame> get_robomas_frame(const std::uint8_t motor,const std::uint8_t mode, const std::uint8_t temp, const float kp, const float ki, const float kd, const float limitie){
		auto msg = std::make_unique<robo_messages::RobomasFrame>();
		msg->motor = motor;
		msg->mode = mode;
		msg->temp = temp;
		msg->kp = kp;
		msg->ki = ki;
		msg->kd = kd;
		msg->limitie = limitie;
		return msg;
	}

	inline std::unique_ptr<robo_messages::RobomasTarget> get_robomas_target(const float target){
		auto msg = std::make_unique<robo_messages::RobomasTarget>();
		msg->target = target;
		return msg;
	}
}