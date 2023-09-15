#pragma once

#include <rclcpp/rclcpp.hpp>
// 頼むから名前をどうにかして。君の名はcan_plugins2じゃないだろ。
#include <fake_robomaster_serial_can/msg/robomas_frame.hpp>

namespace nhk2024::independent_steering_n::robomasu_pub
{
	namespace impl
	{
		using fake_robomaster_serial_can::msg::RobomasFrame;
	}

	void init(const rclcpp::Publisher<impl::RobomasFrame>::SharedPtr&)
	{
		/// @todo ここに初期化処理を書く
	}

	// ロボマスを"動作停止"に変更
	void disable_all(const rclcpp::Publisher<impl::RobomasFrame>::SharedPtr&)
	{
		/// @todo
	}

	// 使用する駆動輪全てのロボマスを"速度制御"に変更
	void enable_all(const rclcpp::Publisher<impl::RobomasFrame>::SharedPtr&)
	{
		/// @todo
	}

	// 左前から反時計回りの順で速度指令値が入った配列をとり、ロボマスに送信
	void send_target(const rclcpp::Publisher<impl::RobomasFrame>::SharedPtr&, const float (&)[4])
	{
		/// @todo
	}
}