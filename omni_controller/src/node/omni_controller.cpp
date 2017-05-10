#include "node/omni_controller.h"

#include <vector>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <geometry_msgs/Vector3.h>

OmniController::OmniController(/*std::string topicName*/)
{
    nodeName="controller";
	enable_gamepad = true;
    force_pub = n.advertise<geometry_msgs::Vector3>("control",1);
    enable_control_pub = n.advertise<std_msgs::Bool>("enable_control",1);
    gamepad_sub = n.subscribe("joy", 1, &OmniController::gamepadPublisher, this);
}

void OmniController::gamepadPublisher(const sensor_msgs::Joy::ConstPtr& msg)
{
	if (msg->buttons[6])			 
	{	
		forceOutput.x = 0;
		forceOutput.y = 0;
		forceOutput.z = 0;
		enable_gamepad = true;
	}

	if (!enable_gamepad)
	{
		return;
	}

	if (msg->buttons[7])
	{
		forceOutput.x = 0;
		forceOutput.y = 0;
		forceOutput.z = 0;
		enable_control.data = true;
		enable_control_pub.publish(enable_control);
	}

	forceOutput.x = -msg->axes[0];
	forceOutput.y = msg->axes[1];
	forceOutput.z = ((msg->axes[2] - 1) - (msg->axes[5] - 1))/2;
    force_pub.publish(forceOutput);
}

void OmniController::teleoperation(std::string robot_name)
{
//    teleop_sub = n.subscribe("twist", 1, &OmniController::teleoperationCallback, this);
}

void OmniController::omniEthernetTeleop()
{
    teleoperation("omniEthernet");
}

void OmniController::omniFirewireTeleop()
{
    teleoperation("omniFirewire");
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "omni_controller");
    OmniController omni;
    ros::spin();
    return 0;
}
