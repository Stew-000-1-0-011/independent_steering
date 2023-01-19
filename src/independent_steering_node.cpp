/**
 * @file independent_steering_node.cpp
 * @author Stew
 * @brief N輪独立ステアリングを制御する。
 * spinning制御(回転運動のみ)とcrab制御(並進運動のみ)の2モードを扱う。
 * @version 0.1
 * @date 2022-11-17
 * 
 */

#include <utility>

#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

namespace crs_lib
{
	class IndependentSteeringNode final : public nodelet::Nodelet
	{
		

		void onInit() override
		{}
	};
}