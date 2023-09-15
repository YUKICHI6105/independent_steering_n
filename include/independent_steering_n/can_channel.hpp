#pragma once

#include <cstdint>
#include <optional>
#include <variant>
#include <exception>
#include <concepts>
#include <type_traits>
#include <utility>
#include <chrono>

namespace nhk2024::independent_steering_n::can_channel
{
	struct DataField final
	{
		std::uint8_t data[8];
		std::uint8_t dlc;

		static auto make(auto x) noexcept -> DataField
		requires (sizeof(std::remove_cvref_t<decltype(x)>) <= 8) && std::is_trivially_copyable_v<decltype(x)>
		{
			DataField ret;
			std::memcpy(ret.data, &x, sizeof(x));
			ret.dlc = sizeof(x);
			return ret;
		}
	};

	// **NOT THREAD SAFE**
	// do not need to implement copy ctor.
	template<class T>
	concept tx_channel = std::move_constructible<T> && std::destructible<T> && requires(T mut, const DataField data)
	{
		// non blocking
		{mut.send(data)} noexcept;
	};

	// **NOT THREAD SAFE**
	// do not need to implement copy ctor.
	template<class T>
	concept rx_channel = std::move_constructible<T> && std::destructible<T> && requires(T mut, const std::chrono::microseconds freshness)
	{
		// non blocking
		{mut.receive(freshness)} noexcept -> std::same_as<std::optional<DataField>>;
	};

	// connection-oriented comunication
	// 1. **NOT THREAD SAFE**. Only have one thread on the thread; others MUST NOT have reference to it.
	// 2. Frames must be received **in the same order they were sent**.
	// Because of these requirements, we can abort transmissions, send frames to each other in order.
	// I don't know of any case where we would want to communicate asynchronously with a single CAN id.
	// So it will not be such a strict requirement.
	namespace connection_oriented
	{
		struct DataField final
		{
			std::uint8_t data[7];
			std::uint8_t dlc;

			static auto make(auto x) noexcept -> DataField
			requires (sizeof(std::remove_cvref_t<decltype(x)>) <= 7) && std::is_trivially_copyable_v<decltype(x)>
			{
				DataField ret;
				std::memcpy(ret.data, &x, sizeof(x));
				ret.dlc = sizeof(x);
				return ret;
			}
		};

		// template<class T>
		// concept pipe = std::move_constructible<T> && std::destructible<T> && requires(T mut, const DataField data, const std::chrono::microseconds timeout)
		// {
		// 	{T::make()}
		// 	{mut.send(data, timeout)} noexcept -> monadic_future::future;
		// 	{mut.}
		// };
	}

	// The tx_id/rx_id is the id of the one that is itself the sender/receiver.
	template<class T>
	concept channel_manager = std::move_constructible<T> && std::destructible<T> && requires(T mut, const std::uint32_t tx_id, const std::uint32_t rx_id)
	{
		typename T::TxChannel;
		typename T::RxChannel;
		typename T::Pipe;
		{mut.tx(tx_id)} noexcept;
		{mut.rx(rx_id)} noexcept;
		// {mut.pipe(tx_id, rx_id)} noexcept;
		{[]<class T2>(std::optional<T2>) -> T2 {return std::declval<T>();}(mut.tx(tx_id))} -> tx_channel;
		{[]<class T2>(std::optional<T2>) -> T2 {return std::declval<T>();}(mut.rx(rx_id))} -> rx_channel;
		// {[]<class T2>(std::optional<T2>) -> T2 {return std::declval<T>();}(mut.pipe(id))} -> connection_oriented::pipe;
	};
}