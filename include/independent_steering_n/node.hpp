#pragma once

#include <cstdint>
#include <cstring>
#include <cmath>
#include <atomic>
#include <mutex>
#include <optional>
#include <utility>
#include <array>
#include <future>
#include <chrono>
#include <functional>
#include <stdexcept>
#include <numbers>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <can_plugins2/msg/frame.hpp>
#include <fake_robomaster_serial_can/msg/robomas_frame.hpp>

#include "utility.hpp"
#include "shirasu.hpp"
#include "robomasu_pub.hpp"
#include "gearbox.hpp"
#include "steering_wheel.hpp"
#include "control_mode.hpp"

using namespace std::chrono_literals;

namespace nhk2024::independent_steering_n::node
{
	namespace impl
	{
		struct Vec2 final
		{
			double x;
			double y;
		};

		struct Twist final
		{
			Vec2 linear;
			double yaw;
		};

		template<class T, class U>
		using OptPair = std::optional<std::pair<T, U>>;

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
		};
	}

	class Node final : public rclcpp::Node
	{
		rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr can_tx;
		rclcpp::Publisher<fake_robomaster_serial_can::msg::RobomasFrame>::SharedPtr robomas_pub;

		std::atomic<impl::Twist> body_twist;
		control_mode::ControlMode mode;
		std::mutex mode_mutex;
		can_plugins2_channel::ChannelManager can_manager;
		std::array<impl::AssembledWheel, 4> wheels;
		std::array<shirasu::Shirasu, 4> shirasus;

		rclcpp::TimerBase::SharedPtr timer{};
		rclcpp::Subscription<can_plugins2::msg::Frame>::SharedPtr can_rx{};
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr body_twist_sub{};
		rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr control_mode_sub{};

		static constexpr auto initialize_wheels() noexcept
		{
			/// @todo 設定
			constexpr rational::Rational steer_ratio{1, 1};
			constexpr rational::Rational drive_ratio{1, 1};

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
			constexpr std::uint32_t steer_id = 0x100;

			std::array<std::optional<shirasu::Shirasu>, 4> shirasu_opt{};

			for(std::uint32_t i = 0; i < 4; ++i)
			{
				shirasu_opt[i] = shirasu::Shirasu::make(can_manager, steer_id + 0x10 * i);
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
		Node(const rclcpp::NodeOptions& options):
			rclcpp::Node("independent_steering_n", options),
			can_tx{this->create_publisher<can_plugins2::msg::Frame>("can_tx", 100)},
			robomas_pub{this->create_publisher<fake_robomaster_serial_can::msg::RobomasFrame>("robomasu_tx", 100)},
			body_twist{impl::Twist{}},
			mode{control_mode::ControlMode::disable},
			mode_mutex{},
			can_manager{can_plugins2_channel::ChannelManager::make(can_tx, this->get_logger())},
			wheels{initialize_wheels()},
			shirasus{initialize_shirasus(can_manager)}
		{
			timer = this->create_wall_timer(10ms, std::bind(&Node::timer_callback, this));
			can_rx = this->create_subscription<can_plugins2::msg::Frame>("can_rx", 100, std::bind(&Node::can_rx_callback, this, std::placeholders::_1));
			body_twist_sub = this->create_subscription<geometry_msgs::msg::Twist>("body_twist", 1, std::bind(&Node::body_twist_callback, this, std::placeholders::_1));
			control_mode_sub = this->create_subscription<std_msgs::msg::UInt8>("control_mode", 100, std::bind(&Node::control_mode_callback, this, std::placeholders::_1));
		}

		void timer_callback()
		{
			using control_mode::ControlMode;

			auto twist = body_twist.load();

			{
				std::lock_guard lck{mode_mutex};

				switch(mode)
				{
					case ControlMode::disable:
					break;
					
					case ControlMode::crab:
					{
						const double theta = std::atan2(twist.linear.y, twist.linear.x);
						const double v = std::sqrt(twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y);
						float drive_targets[4]{};
						for(int i = 0; i < 4; ++i)
						{
							const auto [steer_target, drive_target] = wheels[i].inverse(theta, v);
							shirasus[i].send_target(steer_target);
							drive_targets[i] = drive_target;
						}
						robomasu_pub::send_target(robomas_pub, drive_targets);
					}
					break;

					case ControlMode::spinning:
					{
						float drive_targets[4]{};
						for(int i = 0; i < 4; ++i)
						{
							const auto [steer_target, drive_target] = wheels[i].inverse(wheels[i].zero_angle, twist.yaw);
							shirasus[i].send_target(steer_target);
							drive_targets[i] = drive_target;
						}
						robomasu_pub::send_target(robomas_pub, drive_targets);
					}
					break;
				}
			}
		}

		void can_rx_callback(const can_plugins2::msg::Frame::SharedPtr msg)
		{
			can_manager.receive(msg);
		}

		void body_twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
		{
			const auto linear = impl::Vec2{msg->linear.x, msg->linear.y};
			const double angular = msg->angular.z;
			body_twist.store(impl::Twist{linear, angular});
		}

		void control_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg)
		{
			using control_mode::ControlMode;

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
							robomasu_pub::disable_all(robomas_pub);
							break;
						
						case ControlMode::crab:
							for(auto& steer : shirasus)
							{
								steer.send_command<shirasu::State::velocity>();
							}
							robomasu_pub::enable_all(robomas_pub);
							break;

						case ControlMode::spinning:
							for(auto& steer : shirasus)
							{
								steer.send_command<shirasu::State::velocity>();
							}
							robomasu_pub::enable_all(robomas_pub);
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
	};
}