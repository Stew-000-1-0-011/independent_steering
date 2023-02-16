/**
 * @file xml_rpc.hpp
 * @author Stew (you@domain.com)
 * @brief pragma pack前提のXML-RPCのシリアライザ。
 * @version 0.1
 * @date 2023-02-16
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include <tuple>
#include <concepts>

#include "string_literal.hpp"
#include "std_type.hpp"
#include "utility.hpp"

namespace CRSLib
{
	namespace XmlRpc
	{
		namespace Implement
		{
			struct ArrayMarker{};
			struct StructMarker{};
			struct ItemMarker{};
		}

		template<class T>
		concept is_array = std::derived_from<T, Implement::ArrayMarker>;
		template<class T>
		concept is_struct = std::derived_from<T, Implement::StructMarker>;
		template<class T>
		concept is_item = std::derived_from<T, Implement::ItemMarker>;

		enum Primitive
		{
			Bool,
			Int,
			Double,
			String,
			DateTime,
			Base64
		};

		template<class T>
		concept is_type = std::same_as<T, Primitive> || is_array<T> || is_struct<T>;

		template<is_type auto ... types_>
		struct ArrayType final : Implement::ArrayMarker
		{
			static constexpr std::tuple<decltype(types_) ...> types{};
		};

		// 仕様的には実質tuple。
		// でも普通に使うなら、array<T>ないしarray<variant<T_like1, T_like2, ...>>だと思う。
		// そこで...
		template<is_type auto ... types>
		ArrayType<types ...> array{};

		template<is_string_literal auto identifier_, is_type auto type_>
		struct Item final : Implement::ItemMarker
		{
			static constexpr auto identifier = identifier_;
			static constexpr auto type = type_;
		};

		template<is_string_literal auto identifier, is_type auto type>
		Item<identifier, type> item{};

		template<is_item auto ... items_>
		struct Structure final : Implement::StructMarker
		{
			static constexpr std::tuple<decltype(items_) ...> items{};
		};

		template<is_item auto ... items>
		Structure<items ...> structure{};

		

		template<class CharT, size_t n>
		using id = CRSLib::StringLiteral<CharT, n>;

		constexpr auto pid_t = structure
		<
			item
			<
				id{"p"},
				Double
			>,
			item
			<
				id{"i"},
				Double
			>,
			item
			<
				id{"d"},
				Double
			>
		>;

		constexpr auto motor_t = structure
		<
			item
			<
				id{"id"},
				Int
			>,
			item
			<
				id{"pid"},
				pid_t
			>
		>;

		constexpr auto config_item = item
		<
			id{"config"},
			structure
			<
				item
				<
					id{"limit"},
					Double
				>,
				item
				<
					id{"motors"},
					array
					<
						motor_t,
						motor_t,
						motor_t,
						motor_t
					>
				>
			>
		>;
	}
}