#pragma once

#include <utility>
#include <vector>
#include <functional>
#include <algorithm>

#include "std_type.hpp"

namespace crs_lib::adhoc_can_plugins2
{

	class CallbackManager final
	{
		using Callback = void (const char *const) noexcept;

		struct IdAndCallback final
		{
			u32 id;
			std::function<Callback> callback;
		};

		std::vector<IdAndCallback> id_callbacks{};

		// can_plugins2内で呼び出すことを想定した関数オブジェクト。
		struct CanPluginsCallback final
		{
			std::vector<IdAndCallback> sorted_id_callbacks;

		public:
			CanPluginsCallback(std::vector<IdAndCallback>&& sorted_id_callbacks) noexcept:
				sorted_id_callbacks{std::move(sorted_id_callbacks)}
			{}

			void operator()(const u32 id, const char *const data) noexcept
			{
				for(const auto& id_and_callback : sorted_id_callbacks)
				{
					if(id == id_and_callback.id)
					{
						id_and_callback.callback(data);
					}
					else if(id > id_and_callback.id) return;
				}
			}
		};

	public:
		void push(const u32 id, std::function<Callback>&& callback)
		{
			id_callbacks.emplace_back({id, std::move(callback)});
		}

		CanPluginsCallback make_can_plugins_callback() noexcept
		{
			std::sort(id_callbacks, {}, [](const auto x){return x.id;});
			return {std::move(id_callbacks)};
		}
	};
}