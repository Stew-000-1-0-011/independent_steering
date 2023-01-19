#pragma once

#include <cstring>
#include <bit>
#include <type_traits>
#include <utility>

#include <CRSLib/include/std_int.hpp>

namespace crs_lib::can_plugins2
{
	using namespace CRSLib::IntegerTypes;

	constexpr u32 can_mtu = 8;

	template<class T>
	concept BeAbleToPackC = sizeof(T) <= can_mtu && std::is_trivially_copyable_v<T>;

	// DataField型へ変換
	template<std::endian endian, BeAbleToPackC T>
	inline void pack(void *const buffer, const T& value) noexcept
	{
		std::memcpy(buffer, &value, sizeof(T));

		if constexpr(std::endian::native != endian)
		{
			for(unsigned int i = 0; i < sizeof(T) / 2; ++i)
			{
				std::swap(static_cast<char *>(buffer)[i], static_cast<char *>(buffer)[sizeof(T) - 1 - i]);
			}
		}
	}

	// DataField型から逆変換
	template<std::endian endian, BeAbleToPackC T>
	inline T unpack(const void *const buffer) noexcept
	{
		if constexpr(std::endian::native != endian)
		{
			char tmp[can_mtu];
			for(unsigned int i = 0; i < sizeof(T); ++i)
			{
				tmp[i] = static_cast<const char *>(buffer)[sizeof(T) - 1 - i];
			}

			// 確か大丈夫だったはず(char *からは任意の型にアクセスできるんじゃないっけ？)
			return *reinterpret_cast<const T *>(tmp);
		}
		else
		{
			return *reinterpret_cast<const T *>(buffer);
		}
	}
}
