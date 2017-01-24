#pragma once

#include <vector>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include <geometry_msgs/Vector3.h>
#include <keyboard/Key.h>
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
	ros::Subscriber keyboard_sub;
	ros::Subscriber gamepad_sub;
	bool enable_gamepad;
	bool enable_keyboard;

public:

    OmniController(/*std::string topicName = "keyboard/keydown"*/);

	void keyboardPublisher(const keyboard::Key::ConstPtr& msg);

	void gamepadPublisher(const sensor_msgs::Joy::ConstPtr& msg);

	const std::string getNodeName()
	{
		return this->nodeName;
	}
};
