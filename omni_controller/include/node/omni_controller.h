#pragma once

#include <vector>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
#include <map>

class OmniController
{

private:
    std::vector<int> lenght;
	std::string nodeName;
	std::map<std::string, int> controllerMap;
	ros::Publisher force_pub;
	ros::Publisher enable_control_pub;
	std_msgs::Bool enable_control;
	geometry_msgs::Vector3 forceOutput;
    ros::NodeHandle n;
	ros::Subscriber gamepad_sub;
    ros::Subscriber teleop_sub;
	bool enable_gamepad;

public:

    OmniController();

	void gamepadPublisher(const sensor_msgs::Joy::ConstPtr& msg);

    /**
     * @brief teleoperation Method used to teleoperate other robots using the omni device. To use it, pass the desired teleoperation robot name.
     * Note that the robot must have the default topic names, i.e. robot_name/joint_states, robot_name/twist, etc.
     * @param robot_name String correspondig to your robot prefix in the ROS topics.
     */
    void teleoperation(std::string robot_name);

    void omniEthernetTeleop();

    void omniFirewireTeleop();

	const std::string getNodeName()
	{
		return this->nodeName;
	}
};
