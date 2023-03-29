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
#include <string_view>
#include <vector>
#include <variant>
#include <algorithm>

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
			struct StructureMarker{};
			struct ItemMarker{};
			struct IdentifierMarker{};
			struct DigitIdentifierMarker : IdentifierMarker{};
			struct StringIdentifierMarker : IdentifierMarker{};
		}

		template<class T>
		concept is_array = std::derived_from<T, Implement::ArrayMarker>;
		template<class T>
		concept is_structure = std::derived_from<T, Implement::StructureMarker>;
		template<class T>
		concept is_item = std::derived_from<T, Implement::ItemMarker>;
		template<class T>
		concept is_identifier = std::derived_from<T, Implement::IdentifierMarker>;
		template<class T>
		concept is_digit_identifier = std::derived_from<T, Implement::DigitIdentifierMarker>;
		template<class T>
		concept is_string_identifier = std::derived_from<T, Implement::StringIdentifierMarker>;

		template<auto identifier_>
		struct Identifier;

		template<size_t identifier_>
		struct DigitIdentifier final : Implement::DigitIdentifierMarker
		{
			static constexpr auto identifier = identifier_;
		};

		template<is_string_literal auto identifier_>
		struct StringIdentifier final : Implement::StringIdentifierMarker
		{
			static constexpr auto identifier = identifier_;
		};

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

		template<is_string_identifier auto identifier_, is_type auto type_>
		struct Item final : Implement::ItemMarker
		{
			static constexpr auto identifier = identifier_;
			static constexpr auto type = type_;
		};

		template<is_string_identifier auto identifier, is_type auto type>
		Item<identifier, type> item{};

		template<is_item auto ... items_>
		struct Structure final : Implement::StructureMarker
		{
			static constexpr std::tuple<decltype(items_) ...> items{};
		};

		template<is_item auto ... items>
		Structure<items ...> structure{};

		namespace Implement
		{
			template<is_string_identifier identifier, is_item Head, is_item ... Items>
			consteval auto search_type_helper(const size_t index, const Head&, const Items& ... items)
			{
				if constexpr(Head::identifier == identifier)
				{
					return index;
				}
				else
				{
					return search_type(index + 1, items ...);
				}
			}

			template<class T = void>
			consteval auto search_type_helper()
			{
				if([]{return false;}())
				{
					static_assert("No element that match the identifier in items, or items is empty.");
				}
			}

			template<is_string_identifier identifier, is_item ... Items>
			consteval auto search_type(const std::tuple<Items ...>& items)
			{
				return std::apply([](const auto& ... args) consteval {return search_type_helper(0, args ...);}, items);
			}
		}

		template<class Parent, is_type auto type>
		class Variable final
		{
			std::variant<size_t, std::string_view> name;

			static auto make_array_value(Parent& parent)
			requires is_array<decltype(type)>
			{
				return []<size_t ... indices>(Parent& parent, std::index_sequence<indices>)
				{
					return std::tuple{Variable<Parent, std::get<indices>(type.types)>{indices, parent} ...};
				}(parent, std::make_index_sequence<std::tuple_size_v<decltype(type.types)>>());
			}

			static auto make_structure_value(Parent& parent)
			{
				return []<size_t ... indices>(Parent& parent, std::index_sequence<indices>)
				{
					return std::tuple{Variable<Parent, std::get<indices>(type.items).type>{std::get<indices>(type.items).identifier, parent} ...};
				}(parent, std::make_index_sequence<std::tuple_size_v<decltype(type.items)>>());
			}

		public:
			 v;

			Variable(const decltype(name)& name, Parent& parent)
			requires (!is_array<decltype(type)>) && (!is_structure<decltype(type)>):
				name{name},
				v{parent.template get_value<type>(name)}
			{}

			Variable(const decltype(name)& name, Parent& parent)
			requires is_array<decltype(type)>:
				name{name},
				v{make_array_value(parent)}
			{}

			Variable(const decltype(name)& name, Parent& parent)
			requires is_structure<decltype(type)>:
				name{name},
				v{make_structure_value(parent)}
			{}

			Varibale(Variable&&) = default;
			Variable(const Variable&) = delete;

			// for array.
			auto& operator/(is_digit_identifier auto index)
			requires is_array<decltype(type)>
			{
				return std::get<index.identifier>(v);
			}

			// for structure.
			auto operator/(is_string_identifier auto member)
			requires is_structure<decltype(type)>
			{
				return std::get<Implement::search_type<member.identifier>(type.items)>(v);
			}
		};

		// template<class CharT, size_t n>
		// using id = CRSLib::StringLiteral<CharT, n>;

		// constexpr auto pid_t = structure
		// <
		// 	item
		// 	<
		// 		id{"p"},
		// 		Double
		// 	>,
		// 	item
		// 	<
		// 		id{"i"},
		// 		Double
		// 	>,
		// 	item
		// 	<
		// 		id{"d"},
		// 		Double
		// 	>
		// >;

		// constexpr auto motor_t = structure
		// <
		// 	item
		// 	<
		// 		id{"id"},
		// 		Int
		// 	>,
		// 	item
		// 	<
		// 		id{"pid"},
		// 		pid_t
		// 	>
		// >;

		// constexpr auto config_item = item
		// <
		// 	id{"config"},
		// 	structure
		// 	<
		// 		item
		// 		<
		// 			id{"limit"},
		// 			Double
		// 		>,
		// 		item
		// 		<
		// 			id{"motors"},
		// 			array
		// 			<
		// 				motor_t,
		// 				motor_t,
		// 				motor_t,
		// 				motor_t
		// 			>
		// 		>
		// 	>
		// >;
	}
}