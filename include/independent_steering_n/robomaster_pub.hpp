#pragma once

#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <can_plugins2/msg/robomas_frame.hpp>
//#include <robomas_utils.hpp>
#include "../include/independent_steering_n/robomas_utils.hpp"

namespace nhk2024::independent_steering_n::robomaster_pub
{
	namespace impl
	{
		using can_plugins2::msg::RobomasFrame;
		std::optional<rclcpp::Logger> logger{};
	}

	void init(rclcpp::Logger&& logger, const rclcpp::Publisher<impl::RobomasFrame>::SharedPtr& robomas_pub)
	{
		impl::logger = std::move(logger);
		/// @todo ここに初期化処理を書く
		robomas_pub->publish(get_robomas_frame(0,0,50,30.0,22.0,5.0,1000.0));
      	robomas_pub->publish(get_robomas_frame(1,0,50,30.0,22.0,5.0,1000.0));
      	robomas_pub->publish(get_robomas_frame(2,0,50,30.0,22.0,5.0,1000.0));
      	robomas_pub->publish(get_robomas_frame(3,0,50,30.0,22.0,5.0,1000.0));
		RCLCPP_INFO(*impl::logger, "robomaster_pub::init.");
	}

	// ロボマスを"動作停止"に変更
	void disable_all(const rclcpp::Publisher<impl::RobomasFrame>::SharedPtr& robomas_pub)
	{
		/// @todo
		robomas_pub->publish(get_robomas_frame(0,0,50,30.0,22.0,5.0,1000.0));
      	robomas_pub->publish(get_robomas_frame(1,0,50,30.0,22.0,5.0,1000.0));
      	robomas_pub->publish(get_robomas_frame(2,0,50,30.0,22.0,5.0,1000.0));
      	robomas_pub->publish(get_robomas_frame(3,0,50,30.0,22.0,5.0,1000.0));
      	// publisher_->publish(get_frame(pubsub::bidNumber.upperLeftsteering,static_cast<uint8_t>(0)));
      	// publisher_->publish(get_frame(pubsub::bidNumber.upperRightsteering,static_cast<uint8_t>(0)));
      	// publisher_->publish(get_frame(pubsub::bidNumber.lowerLeftsteering,static_cast<uint8_t>(0)));
      	// publisher_->publish(get_frame(pubsub::bidNumber.lowerRightsteering,static_cast<uint8_t>(0)));
		RCLCPP_INFO(*impl::logger, "robomaster_pub::disable_all.");
	}

	// 使用する駆動輪全てのロボマスを"速度制御"に変更
	void enable_all(const rclcpp::Publisher<impl::RobomasFrame>::SharedPtr& robomas_pub)
	{
		/// @todo
		robomas_pub->publish(get_robomas_frame(0,1,50,30.0,22.0,5.0,1000.0));
      	robomas_pub->publish(get_robomas_frame(1,1,50,30.0,22.0,5.0,1000.0));
      	robomas_pub->publish(get_robomas_frame(2,1,50,30.0,22.0,5.0,1000.0));
      	robomas_pub->publish(get_robomas_frame(3,1,50,30.0,22.0,5.0,1000.0));
      	// publisher_->publish(get_frame(pubsub::bidNumber.upperLeftsteering,static_cast<uint8_t>(6)));
      	// publisher_->publish(get_frame(pubsub::bidNumber.upperRightsteering,static_cast<uint8_t>(6)));
      	// publisher_->publish(get_frame(pubsub::bidNumber.lowerLeftsteering,static_cast<uint8_t>(6)));
      	// publisher_->publish(get_frame(pubsub::bidNumber.lowerRightsteering,static_cast<uint8_t>(6)));
		RCLCPP_INFO(*impl::logger, "robomaster_pub::enable_all.");
	}

	// 左前から反時計回りの順で速度指令値が入った配列をとり、ロボマスに送信
	void send_target(const rclcpp::Publisher<can_plugins2::msg::RobomasTarget>::SharedPtr& robomas_target1, 
					const rclcpp::Publisher<can_plugins2::msg::RobomasTarget>::SharedPtr& robomas_target2,
					const rclcpp::Publisher<can_plugins2::msg::RobomasTarget>::SharedPtr& robomas_target3,
					const rclcpp::Publisher<can_plugins2::msg::RobomasTarget>::SharedPtr& robomas_target4,
					 const float (&targets)[4])
	{
		/// @todo
		robomas_target1->publish(get_robomas_target(targets[0]));
        robomas_target2->publish(get_robomas_target(targets[1]));
        robomas_target3->publish(get_robomas_target(targets[2]));
        robomas_target4->publish(get_robomas_target(targets[3]));
		std::string s{};
		for(const auto target : targets)
		{
			s += std::to_string(target) + ", ";
		}
		RCLCPP_INFO(*impl::logger, "robomaster_pub::send_target: %s", s.c_str());
	}
}