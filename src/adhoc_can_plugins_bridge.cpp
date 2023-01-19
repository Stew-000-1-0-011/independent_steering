#include <vector>
#include <string>

#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>

#include <CanFrame.h>

#include <crs_lib/rosparam_util.hpp>

namespace crs_lib
{
	class CanPluginsBridgeNode final : public nodelet::Nodelet
	{
		std::vector<ros::Publisher> pubs{};

		void onInit() override
		{
			auto nh = Nodelet::getMTNodeHandle();
			auto tx_ids = CRSLib::RosparamUtil::get_param(nh, "tx_ids");
			
			for(int i = 0; ; ++i)
			{
				auto tx_id = CRSLib::RosparamUtil::get_param(tx_ids, i);
				if(!tx_id) break;
				else
				{
					tx_ids.emplace(nh.advertise<independent_steering::CanFrame>("can" + std::to_string(read_param(tx_id, ), ))
				}
			}
			
		}
	};
}