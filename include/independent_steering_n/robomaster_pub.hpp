#pragma once

#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <fake_robomaster_serial_can/msg/robomas_frame.hpp>

namespace nhk2024::independent_steering_n::robomaster_pub
{
	namespace impl
	{
		using fake_robomaster_serial_can::msg::RobomasFrame;
		std::optional<rclcpp::Logger> logger{};
	}

	void init(rclcpp::Logger&& logger, const rclcpp::Publisher<impl::RobomasFrame>::SharedPtr&)
	{
		impl::logger = std::move(logger);
		/// @todo ここに初期化処理を書く
		RCLCPP_INFO(*impl::logger, "robomaster_pub::init.");
	}

	// ロボマスを"動作停止"に変更
	void disable_all(const rclcpp::Publisher<impl::RobomasFrame>::SharedPtr&)
	{
		/// @todo
		RCLCPP_INFO(*impl::logger, "robomaster_pub::disable_all.");
	}

	// 使用する駆動輪全てのロボマスを"速度制御"に変更
	void enable_all(const rclcpp::Publisher<impl::RobomasFrame>::SharedPtr&)
	{
		/// @todo
		RCLCPP_INFO(*impl::logger, "robomaster_pub::enable_all.");
	}

	// 左前から反時計回りの順で速度指令値が入った配列をとり、ロボマスに送信
	void send_target(const rclcpp::Publisher<impl::RobomasFrame>::SharedPtr&, const float (&targets)[4])
	{
		/// @todo
		std::string s{};
		for(const auto target : targets)
		{
			s += std::to_string(target) + ", ";
		}
		RCLCPP_INFO(*impl::logger, "robomaster_pub::send_target: %s", s.c_str());
	}
}