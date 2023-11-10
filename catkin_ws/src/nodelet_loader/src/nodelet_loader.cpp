#ifndef NODELET_LOADER_
#define NODELET_LOADER_

#include <signal.h>
#include <vector>
#include <ros/ros.h>
#include <nodelet/loader.h>
#include <image_proc/advertisement_checker.h>

nodelet::Loader * nodelet_manager_ptr = nullptr;

void on_shutdown(int signal)
{
	nodelet_manager_ptr->unload("tau/tau_nodelet");
	nodelet_manager_ptr->unload("vn100/vn100_nodelet");
	nodelet_manager_ptr->unload("ptgrey/ptgrey_nodelet");
	ros::shutdown();
}
// Nodelet string = "@(package)/@(nodelet)"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "nodelet_loader", ros::init_options::NoSigintHandler);
	signal(SIGINT, on_shutdown);
	nodelet::Loader nodelet_manager(true); //(false); Don't bring up the manager ROS API
	nodelet_manager_ptr = &nodelet_manager;
	nodelet::M_string remappings;
	nodelet::V_string my_argv;
	std::string node_name = ros::this_node::getName();

	ros::NodeHandle nh;

	std::vector<std::string> nodelet_names;
	nh.getParam("/nodelet_loader/nodelet_names", nodelet_names);

	for(unsigned char i  = 0; i < nodelet_names.size(); i++)
	{
		if(!nodelet_manager.load(nodelet_names[i], nodelet_names[i], remappings, my_argv))
		{
			ROS_ERROR("FAILED TO LOAD %s", nodelet_names[i].c_str());
			ros::shutdown();
			return 0;
		}
		else
		{
			ROS_INFO("LOADED %s", nodelet_names[i].c_str());
		}
	}
	ROS_INFO("NODELETS LOADED SUCCESSFULLY");

	ros::spin();
	return 0;
}

#endif // NODELET_LOADER_