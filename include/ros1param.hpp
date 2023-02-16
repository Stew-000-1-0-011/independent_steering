#pragma once

#include <string_view>

#include <ros/ros.h>

#include "xml_rpc.hpp"

namespace CRSLib::Ros1param
{
	/**
	 * @brief ノードハンドルから値を直接読み出す。
	 * 
	 * @param nh 
	 * @param item 
	 * @param root_name 
	 */
	inline void read(ros::NodeHandle& nh)
	{
		nh.getParam()
	}
}