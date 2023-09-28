#pragma once

#if 0
#include <fake_robomaster_serial_can/msg/robomas_frame.hpp>
#include <fake_robomaster_serial_can/msg/robomas_target.hpp>

namespace nhk2024::independent_steering_n::robo_messages
{
	using namespace fake_robomaster_serial_can::msg;
}

#else
#include <can_plugins2/msg/robomas_frame.hpp>
#include <can_plugins2/msg/robomas_target.hpp>

namespace nhk2024::independent_steering_n::robo_messages
{
	using namespace can_plugins2::msg;
}
#endif