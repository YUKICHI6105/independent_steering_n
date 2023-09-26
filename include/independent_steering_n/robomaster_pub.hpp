#pragma once

#include <optional>

#include <rclcpp/rclcpp.hpp>

#include "robomas_utils.hpp"
#include "not_canplugins2_part.hpp"

namespace nhk2024::independent_steering_n::robomaster_pub
{
	namespace impl
	{
		using robo_messages::RobomasFrame;
		using robo_messages::RobomasTarget;

		using robomas_utils::get_robomas_frame;
		using robomas_utils::get_robomas_target;

		std::optional<rclcpp::Logger> logger{};
	}

	void init(rclcpp::Logger&& logger, const rclcpp::Publisher<impl::RobomasFrame>::SharedPtr& robomas_pub)
	{
		impl::logger = std::move(logger);
		/// @todo ここに初期化処理を書く
		robomas_pub->publish(impl::get_robomas_frame(0,0,50,30.0,22.0,5.0,1000.0));
      	robomas_pub->publish(impl::get_robomas_frame(1,0,50,30.0,22.0,5.0,1000.0));
      	robomas_pub->publish(impl::get_robomas_frame(2,0,50,30.0,22.0,5.0,1000.0));
      	robomas_pub->publish(impl::get_robomas_frame(3,0,50,30.0,22.0,5.0,1000.0));
		RCLCPP_INFO(*impl::logger, "robomaster_pub::init.");
	}

	// ロボマスを"動作停止"に変更
	void disable_all(const rclcpp::Publisher<impl::RobomasFrame>::SharedPtr& robomas_pub)
	{
		/// @todo
		robomas_pub->publish(impl::get_robomas_frame(0,0,50,30.0,22.0,5.0,1000.0));
      	robomas_pub->publish(impl::get_robomas_frame(1,0,50,30.0,22.0,5.0,1000.0));
      	robomas_pub->publish(impl::get_robomas_frame(2,0,50,30.0,22.0,5.0,1000.0));
      	robomas_pub->publish(impl::get_robomas_frame(3,0,50,30.0,22.0,5.0,1000.0));
		RCLCPP_INFO(*impl::logger, "robomaster_pub::disable_all.");
	}

	// 使用する駆動輪全てのロボマスを"速度制御"に変更
	void enable_all(const rclcpp::Publisher<impl::RobomasFrame>::SharedPtr& robomas_pub)
	{
		/// @todo
		robomas_pub->publish(impl::get_robomas_frame(0,1,50,30.0,22.0,5.0,1000.0));
      	robomas_pub->publish(impl::get_robomas_frame(1,1,50,30.0,22.0,5.0,1000.0));
      	robomas_pub->publish(impl::get_robomas_frame(2,1,50,30.0,22.0,5.0,1000.0));
      	robomas_pub->publish(impl::get_robomas_frame(3,1,50,30.0,22.0,5.0,1000.0));
		RCLCPP_INFO(*impl::logger, "robomaster_pub::enable_all.");
	}

	// 左前から反時計回りの順で速度指令値が入った配列をとり、ロボマスに送信
	void send_target(const rclcpp::Publisher<impl::RobomasTarget>::SharedPtr& robomas_target1, 
					const rclcpp::Publisher<impl::RobomasTarget>::SharedPtr& robomas_target2,
					const rclcpp::Publisher<impl::RobomasTarget>::SharedPtr& robomas_target3,
					const rclcpp::Publisher<impl::RobomasTarget>::SharedPtr& robomas_target4,
					 const float (&targets)[4])
	{
		/// @todo
		robomas_target1->publish(impl::get_robomas_target(targets[0]));
        robomas_target2->publish(impl::get_robomas_target(targets[1]));
        robomas_target3->publish(impl::get_robomas_target(targets[2]));
        robomas_target4->publish(impl::get_robomas_target(targets[3]));
		std::string s{};
		for(const auto target : targets)
		{
			s += std::to_string(target) + ", ";
		}
		RCLCPP_INFO(*impl::logger, "robomaster_pub::send_target: %s", s.c_str());
	}
}