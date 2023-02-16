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

#include <ctime>
#include <string>
#include <variant>
#include <vector>

#include <CRSLib/include/boxed_object.hpp>

#include "std_type.hpp"
#include "utility.hpp"

namespace CRSLib
{
	namespace XmlRpc
	{
		enum class PrimitiveType
		{
			Bool,
			Int,
			Double,
			String,
			DateTime,
			Base64
		};

		struct ArrayType;
		struct StructType;
		using Type = std::variant<PrimitiveType, BoxedObject<ArrayType>, BoxedObject<StructType>>;

		struct ArrayType final
		{
			std::vector<PrimitiveType> types;
		};

		struct ItemDeclaration final
		{
			std::string identifier;
			Type type;
		};

		struct StructType final
		{
			std::vector<ItemDeclaration> items;
		};

		inline auto def(const std::string& identifier, const std::variant<PrimitiveType, ArrayType>& prim_or_array) -> ItemDeclaration
		{
			return std::visit
			(
				[](const auto& e)
				{
					return ItemDeclaration{identifier, e};
				},
				prim_or_array
			);
		}

		inline auto def(const std::string& identifier)
		{
			return [identifier](const cvref_same<ItemDeclaration> auto& ... structures) -> ItemDeclaration
			{
				return ItemDeclaration{std::move(identifier), StructType{{structures ...}}};
			}
		}
	}
}