/**
 * @file node.hpp
 * @author Stew (you@domain.com)
 * @brief indenependent_steering_nのノード。ヘッダに入れとけば実行ファイルにするのもコンポーネント化するのも楽。
 * @version 0.1
 * @date 2023-09-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include <cstdint>
#include <cmath>

#include <chrono>
#include <optional>
#include <utility>
#include <atomic>
#include <mutex>
#include <array>
#include <numbers>
#include <stdexcept>
#include <string_view>
#include <string>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <can_plugins2/msg/frame.hpp>
#include <independent_steering_n/msg/linear_velocity.hpp>
#include <independent_steering_n/msg/angular_velocity.hpp>

#include "utility.hpp"
#include "shirasu.hpp"
#include "robomaster_pub.hpp"
#include "gearbox.hpp"
#include "steering_wheel.hpp"
#include "control_mode.hpp"
#include "not_canplugins2_part.hpp"

namespace nhk2024::independent_steering_n::node
{
	using namespace std::chrono_literals;

	constexpr double deadzone = 0.05;

	namespace impl
	{
		struct Vec2 final
		{
			double x;
			double y;
		};

		struct AssembledWheel final
		{
			const gearbox::Gearedbox steer_gb;
			const gearbox::Gearedbox drive_gb;
			steering_wheel::SteeringWheel steering_wheel;
			const double zero_angle;  // 操舵角0度のときの機体座標上でのホイールの向き (rad) (ホイールの位置ベクトルに直交している)

			static constexpr auto make(const double zero_angle, const rational::Rational& steer, const rational::Rational& drive) noexcept -> AssembledWheel
			{
				return AssembledWheel{gearbox::Gearedbox{std::move(steer)}, gearbox::Gearedbox{std::move(drive)}, steering_wheel::SteeringWheel{}, zero_angle};
			}

			auto inverse(const double steering_angle, const double driving_speed) noexcept -> std::pair<double, double>
			{
				const auto [angle, speed] = steering_wheel.inverse(steering_angle - zero_angle, driving_speed);
				return {steer_gb.inverse(angle), drive_gb.inverse(speed)};
			}

			auto stop() noexcept -> double
			{
				return steer_gb.inverse(steering_wheel.stop());
			}
		};
	}

	class Node final : public rclcpp::Node
	{
		// publisher
		rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr can_tx;
		rclcpp::Publisher<robo_messages::RobomasFrame>::SharedPtr robomas_pub;
		rclcpp::Publisher<robo_messages::RobomasTarget>::SharedPtr robomas_pub1_;
    	rclcpp::Publisher<robo_messages::RobomasTarget>::SharedPtr robomas_pub2_;
    	rclcpp::Publisher<robo_messages::RobomasTarget>::SharedPtr robomas_pub3_;
    	rclcpp::Publisher<robo_messages::RobomasTarget>::SharedPtr robomas_pub4_;

		// state of undercarriage
		std::atomic<impl::Vec2> linear_velocity;
		std::atomic<double> angular_velocity;
		control_mode::ControlMode mode;
		std::mutex mode_mutex;

		// object to calculate each motor's target
		std::array<impl::AssembledWheel, 4> wheels;

		// object to send target to each steering shirasu motor via CAN
		can_plugins2_channel::ChannelManager can_manager;
		std::array<shirasu::Shirasu, 4> shirasus;
		// (to send target, you can use robomaster_pub::*)

		// timer and subscriber
		rclcpp::TimerBase::SharedPtr timer{};
		rclcpp::Subscription<can_plugins2::msg::Frame>::SharedPtr can_rx{};
		rclcpp::Subscription<::independent_steering_n::msg::LinearVelocity>::SharedPtr linear_velocity_sub{};
		rclcpp::Subscription<::independent_steering_n::msg::AngularVelocity>::SharedPtr angular_velocity_sub{};
		rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr control_mode_sub{};

		static constexpr auto initialize_wheels() noexcept
		{
			/// @todo 設定
			constexpr rational::Rational steer_ratio{2, 5};
			constexpr rational::Rational drive_ratio{1, 10};

			return std::array<impl::AssembledWheel, 4>
			{
				impl::AssembledWheel::make(std::numbers::pi * 3 / 4, steer_ratio, drive_ratio),
				impl::AssembledWheel::make(std::numbers::pi * 5 / 4, steer_ratio, drive_ratio),
				impl::AssembledWheel::make(std::numbers::pi * 7 / 4, steer_ratio, drive_ratio),
				impl::AssembledWheel::make(std::numbers::pi * 1 / 4, steer_ratio, drive_ratio)
			};
		}

		static auto initialize_shirasus(can_plugins2_channel::ChannelManager& can_manager) noexcept(false) -> std::array<shirasu::Shirasu, 4>
		{
			//constexpr std::uint32_t steer_id = 0x100;

			std::array<std::optional<shirasu::Shirasu>, 4> shirasu_opt{};

			shirasu_opt[0] = shirasu::Shirasu::make(can_manager, 0x150);
			shirasu_opt[1] = shirasu::Shirasu::make(can_manager, 0x144);
			shirasu_opt[2] = shirasu::Shirasu::make(can_manager, 0x168);
			shirasu_opt[3] = shirasu::Shirasu::make(can_manager, 0x110);

			for(std::uint32_t i = 0; i < 4; ++i)
			{
				if(!shirasu_opt[i])
				{
					throw std::runtime_error{"failed to make shirasu while initializing shirasus."};
				}
			}

			return std::array<shirasu::Shirasu, 4>
			{
				std::move(*shirasu_opt[0]),
				std::move(*shirasu_opt[1]),
				std::move(*shirasu_opt[2]),
				std::move(*shirasu_opt[3])
			};
		}

		public:
		Node(const std::string_view node_name, const rclcpp::NodeOptions& options):
			rclcpp::Node(std::string(node_name), options),
			can_tx{this->create_publisher<can_plugins2::msg::Frame>("can_tx", 100)},
			robomas_pub{this->create_publisher<robo_messages::RobomasFrame>("robomaster", 100)},
			robomas_pub1_{this->create_publisher<robo_messages::RobomasTarget>("robomas_target1", 10)},
			robomas_pub2_{this->create_publisher<robo_messages::RobomasTarget>("robomas_target2", 10)},
			robomas_pub3_{this->create_publisher<robo_messages::RobomasTarget>("robomas_target3", 10)},
			robomas_pub4_{this->create_publisher<robo_messages::RobomasTarget>("robomas_target4", 10)},
			linear_velocity{},
			angular_velocity{},
			mode{control_mode::ControlMode::disable},
			mode_mutex{},
			wheels{initialize_wheels()},
			can_manager{can_plugins2_channel::ChannelManager::make(can_tx, this->get_logger())},
			shirasus{initialize_shirasus(can_manager)}
		{
			robomaster_pub::init(this->get_logger(), robomas_pub);
			timer = this->create_wall_timer(10ms, std::bind(&Node::timer_callback, this));
			can_rx = this->create_subscription<can_plugins2::msg::Frame>("can_rx", 100, std::bind(&Node::can_rx_callback, this, std::placeholders::_1));
			linear_velocity_sub = this->create_subscription<::independent_steering_n::msg::LinearVelocity>(std::string(node_name) + "/linear_velocity", 1, std::bind(&Node::linear_velocity_callback, this, std::placeholders::_1));
			angular_velocity_sub = this->create_subscription<::independent_steering_n::msg::AngularVelocity>(std::string(node_name) + "/angular_velocity", 1, std::bind(&Node::angular_velocity_callback, this, std::placeholders::_1));
			control_mode_sub = this->create_subscription<std_msgs::msg::UInt8>(std::string(node_name) + "/control_mode", 100, std::bind(&Node::control_mode_callback, this, std::placeholders::_1));
		}

		private:
		void timer_callback()
		{
			using control_mode::ControlMode;

			{
				std::lock_guard lck{mode_mutex};

				switch(mode)
				{
					case ControlMode::disable:
					break;
					
					case ControlMode::crab:
					{
						const auto linear = linear_velocity.load();
						const double theta = std::atan2(linear.y, linear.x);
						const double v = std::sqrt(linear.x * linear.x + linear.y * linear.y);

						float drive_targets[4]{};
						if(v < deadzone) for(int i = 0; i < 4; ++i)
						{
							const auto steer_target = wheels[i].stop();
							shirasus[i].send_target(steer_target);
						}
						else for(int i = 0; i < 4; ++i)
						{
							const auto [steer_target, drive_target] = wheels[i].inverse(theta, v);
							RCLCPP_INFO(this->get_logger(), "steer_target: %f", steer_target);
							shirasus[i].send_target(steer_target);
							drive_targets[i] = drive_target;
						}

						robomaster_pub::send_target(robomas_pub1_, robomas_pub2_, robomas_pub3_, robomas_pub4_, drive_targets);
					}
					break;

					case ControlMode::spinning:
					{
						const auto angular = angular_velocity.load();
						float drive_targets[4]{};
						for(int i = 0; i < 4; ++i)
						{
							const auto [steer_target, drive_target] = wheels[i].inverse(wheels[i].zero_angle, angular);
							RCLCPP_INFO(this->get_logger(), "steer_target: %f", steer_target);
							shirasus[i].send_target(steer_target);
							drive_targets[i] = drive_target;
						}
						robomaster_pub::send_target(robomas_pub1_, robomas_pub2_, robomas_pub3_, robomas_pub4_, drive_targets);
					}
					break;
				}
			}
		}

		void can_rx_callback(const can_plugins2::msg::Frame::ConstSharedPtr msg)
		{
			can_manager.receive(msg);
		}

		void linear_velocity_callback(const ::independent_steering_n::msg::LinearVelocity::ConstSharedPtr msg)
		{
			auto linear = impl::Vec2{msg->x, msg->y};
			linear_velocity.store(std::move(linear));
		}

		void angular_velocity_callback(const ::independent_steering_n::msg::AngularVelocity::ConstSharedPtr msg)
		{
			angular_velocity.store(msg->yaw);
		}

		void control_mode_callback(const std_msgs::msg::UInt8::ConstSharedPtr msg)
		{
			using control_mode::ControlMode;

			clear_velocity();

			auto mode = control_mode::to_control_mode(*msg);
			if(mode)
			{
				{
					std::lock_guard lck{mode_mutex};
					RCLCPP_INFO(this->get_logger(), "control mode changing start: %d", msg->data);
					switch(*mode)
					{
						case ControlMode::disable:
							for(auto& steer : shirasus)
							{
								steer.send_command<shirasu::State::disable>();
							}
							robomaster_pub::disable_all(robomas_pub);
							break;
						
						case ControlMode::crab:
							for(auto& steer : shirasus)
							{
								steer.send_command<shirasu::State::position>();
							}
							robomaster_pub::enable_all(robomas_pub);
							break;

						case ControlMode::spinning:
							for(auto& steer : shirasus)
							{
								steer.send_command<shirasu::State::position>();
							}
							robomaster_pub::enable_all(robomas_pub);
							break;
					}
					RCLCPP_INFO(this->get_logger(), "control mode changing end: %d", msg->data);

					this->mode = *mode;
				}
			}
			else
			{
				RCLCPP_WARN(this->get_logger(), "invalid control mode: %d", msg->data);
			}
		}

		void clear_velocity() noexcept
		{
			linear_velocity.store(impl::Vec2{});
			angular_velocity.store(0.0);
		}
	};
}