#pragma once

#include <cstdint>
#include <optional>
#include <utility>

#include <can_plugins2/msg/frame.hpp>

#include "can_channel.hpp"

namespace nhk2024::independent_steering_n::shirasu
{
	/// @attention shirasu_fwのコードを確認すること。Wikiは罠。ModeとCmdのうち、CANに送るのはCmd。
	/// なので、ModeからCmdに変換してから送ること。
	/// サークルではModeのほうが認知されているので、こちらをベースにした。
	enum class State : std::uint8_t
	{
		disable,
		duty,
		current,
		velocity,
		position,
		homing,
		zero_point,
		indeterminate
	};

	namespace impl
	{
		template<auto x>
		struct ComptimePanic final
		{
			static_assert([]{return false;}());
		};

		template<State state>
		ComptimePanic<state> to_command{};

		template<>
		std::uint8_t to_command<State::disable> = 0x00;
		/// @todo if implemented, change this
		// template<>
		// std::uint8_t to_command<State::duty> = /** @todo */;
		template<>
		std::uint8_t to_command<State::current> = 0x04;
		template<>
		std::uint8_t to_command<State::velocity> = 0x05;
		template<>
		std::uint8_t to_command<State::position> = 0x06;
		template<>
		std::uint8_t to_command<State::homing> = 0x02;
		/// @todo if implemented, change this
		// template<>
		// std::uint8_t to_command<State::tell_state> = /** @todo */;

#if 0
		template<State now, can_channel::channel_manager ChannelManager>
		struct Shirasu
		{
			typename ChannelManager::TxChannel command;
			typename ChannelManager::TxChannel target;
			typename ChannelManager::RxChannel mode_accepted;
		};

		template<shirasu::State next, can_channel::channel_manager ChannelManager>
		using IntoResult = std::future<std::variant<Shirasu<next, ChannelManager>, Shirasu<State::indeterminate, ChannelManager>>>;

		template<State next, State now, can_channel::channel_manager ChannelManager>
		requires (now != next && now != State::indeterminate && next != State::indeterminate)
		inline auto try_into(Shirasu<now, ChannelManager>&& determinate) noexcept -> IntoResult<next, ChannelManager>
		{
			command.send(can_channel::DataField::make(to_command<to_mode<next>>));

			return std::async(std::launch::async, [self = std::move(*this)]{
				using std::chrono_literals;

				auto start = std::chrono::steady_clock::now();
				while(std::chrono::steady_clock::now() - start < 5m) if(const auto data = self.mode_accepted.receive(1ms); data)
				{
					return Shirasu<next, ChannelManager>{std::move(self.command), std::move(self.target), std::move(self.mode_accepted)};
				}

				return Shirasu<State::indeterminate, ChannelManager>{std::move(self.command), std::move(self.target), std::move(self.mode_accepted)};
			});
		}

		template<State next, can_channel::channel_manager ChannelManager>
		requires (next != State::indeterminate)
		inline auto sync_state(Shirasu<State::indeterminate, ChannelManager>&& indeterminate) noexcept -> IntoResult<next, ChannelManager>
		{
			static_assert("not implemented.");
		}

		template<State now, can_channel::channel_manager ChannelManager>
		requires (now != State::disable)
		inline auto force_disable(Shirasu<now, ChannelManager>&& all_shirasu) noexcept -> IntoResult<State::disable, ChannelManager>
		{
			static_assert("not implemented.");
		}

		template<can_channel::channel_manager ChannelManager>
		inline auto make_shirasu(ChannelManager& manager, const std::uint32_t base_id) noexcept -> std::optional<Shirasu<State::disable, ChannelManager>>
		{
			auto command = manager.tx(base_id + 0);
			auto target = manager.tx(base_id + 1);
			auto mode_accepted = manager.rx(base_id + 3);

			if(!command || !target || !mode_accepted)
			{
				return std::nullopt;
			}

			return Shirasu<State::disable, ChannelManager>{std::move(*command), std::move(*target), std::move(*mode_accepted)};
		}

		template<can_channel::channel_manager ChannelManager, State head, State ... tails>
		using ShirasuVariant = std::variant<Shirasu<head, ChannelManager>, Shirasu<tails, ChannelManager>, ...>;
#endif
	}
}

#include "can_plugins2_channel.hpp"

#if 0
namespace nhk2024::independent_steering_n::shirasu
{
	template<State now>
	using Shirasu = impl::Shirasu<now, can_plugins2_channel::ChannelManager>;

	template<State m>
	using IntoResult = impl::IntoResult<m, can_plugins2_channel::ChannelManager>;

	inline auto make_shirasu(can_plugins2_channel::ChannelManager& manager, const std::uint32_t base_id) noexcept -> std::optional<Shirasu<State::disable>>
	{
		return impl::make_shirasu(manager, base_id);
	}

	template<State head, State ... tails>
	using ShirasuVariant = impl::ShirasuVariant<can_plugins2_channel::ChannelManager, head, tails...>;
}
#else
namespace nhk2024::independent_steering_n::shirasu
{
	struct Shirasu
	{
		can_plugins2_channel::ChannelManager::TxChannel command;
		can_plugins2_channel::ChannelManager::TxChannel target;

		template<State next_state>
		void send_command() noexcept
		{
			command.send(can_channel::DataField::make(impl::to_command<next_state>));
		}

		void send_target(const float x) noexcept
		{
			target.send(can_channel::DataField::make(x));
		}

		static auto make(can_plugins2_channel::ChannelManager& manager, const std::uint32_t base_id) noexcept -> std::optional<Shirasu>
		{
			auto command = manager.tx(base_id + 0);
			auto target = manager.tx(base_id + 1);

			if(!command || !target)
			{
				return std::nullopt;
			}

			return Shirasu{std::move(*command), std::move(*target)};
		}
	};
}
#endif
