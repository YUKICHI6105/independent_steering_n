#pragma once

#include <cstdint>
#include <optional>
#include <queue>
#include <utility>
#include <syncstream>
#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <can_plugins2/msg/frame.hpp>

#include "can_channel.hpp"

namespace nhk2024::independent_steering_n::can_plugins2_channel
{
	auto to_msg(const std::uint32_t& id, const can_channel::DataField& data) -> can_plugins2::msg::Frame
	{
		can_plugins2::msg::Frame msg{};

		msg.id = id;
		std::memcpy(msg.data.data(), data.data, data.dlc);
		msg.dlc = data.dlc;
		msg.is_error = false;
		msg.is_extended = false;
		msg.is_rtr = false;

		return msg;
	}

	auto from_msg(const can_plugins2::msg::Frame& msg) -> std::pair<std::uint32_t, can_channel::DataField>
	{
		can_channel::DataField data{};
		std::memcpy(data.data, msg.data.data(), msg.dlc);
		data.dlc = msg.dlc;
		return {msg.id, data};
	}

	namespace impl
	{
		struct TimeStampedData final
		{
			std::chrono::system_clock::time_point time;
			can_channel::DataField data;
		};
	}

	// 例外安全なqueue。
	struct CanListener final
	{
		std::queue<impl::TimeStampedData> received{};

		void push(const can_channel::DataField& data) noexcept
		{
			try
			{
				auto now = std::chrono::system_clock::now();
				received.push(impl::TimeStampedData{std::move(now), data});
			}
			catch(std::exception& e)
			{
				try
				{
					std::osyncstream{std::cerr} << e.what() << std::endl;
				}
				catch(...) {}
			}
		}

		auto pop(const std::chrono::microseconds& freshness) noexcept -> std::optional<can_channel::DataField>
		{
			try
			{
				while(!received.empty())
				{
					auto ret = received.front();
					received.pop();
					if(ret.time - std::chrono::system_clock::now() < freshness) return ret.data;
				}
				
				return std::nullopt;
			}
			catch(const std::exception& e)
			{
				std::cerr << e.what() << '\n';
				return std::nullopt;
			}
		}
	};

	struct ChannelManager final
	{
		struct TxChannel final
		{
			rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr pub;
			rclcpp::Logger logger;
			std::uint32_t id;

			// thread safe (shared_ptrなので)
			void send(const can_channel::DataField& data) noexcept
			{
				try
				{
					pub->publish(to_msg(id, data));
				}
				catch(const std::exception& e)
				{
					try
					{
						std::osyncstream{std::cerr} << e.what() << std::endl;
					}
					catch(...) {}
				}
			}
		};

		struct RxChannel final
		{
			// thread safe (shared_ptrなので)
			std::shared_ptr<CanListener> listener;

			auto receive(const std::chrono::microseconds& freshness) noexcept -> std::optional<can_channel::DataField>
			{
				return listener->pop(freshness);
			}
		};

		rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr pub;
		rclcpp::Logger logger;
		std::unordered_map<std::uint32_t, std::shared_ptr<CanListener>> listeners;

		static auto make(const rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr& pub, const rclcpp::Logger& logger) noexcept -> ChannelManager
		{
			return ChannelManager{pub, logger, {}};
		}

		void receive(const can_plugins2::msg::Frame::ConstSharedPtr& msg) noexcept
		{
			try
			{
				if(listeners.contains(msg->id))
				{
					listeners.at(msg->id)->push(from_msg(*msg).second);
				}
			}
			catch(const std::exception& e)
			{
				try
				{
					std::osyncstream{std::cerr} << e.what() << std::endl;
				}
				catch(...) {}
			}
		}

		auto tx(const std::uint32_t id) noexcept -> std::optional<TxChannel>
		{
			try
			{
				return TxChannel{pub, logger, id};
			}
			catch(const std::exception& e)
			{
				try
				{
					std::osyncstream{std::cerr} << e.what() << std::endl;
				}
				catch(...) {}
				return std::nullopt;
			}
		}

		auto rx(const std::uint32_t id) noexcept -> std::optional<RxChannel>
		{
			try
			{
				if(listeners.contains(id))
				{
					return RxChannel{listeners.at(id)};
				}
				else
				{
					listeners[id] = std::make_shared<CanListener>();
					return RxChannel{listeners.at(id)};
				}
			}
			catch(const std::exception& e)
			{
				try
				{
					std::osyncstream{std::cerr} << e.what() << std::endl;
				}
				catch(...) {}
				return std::nullopt;
			}
		}
	};
}