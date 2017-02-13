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
	enable_gamepad = false;
	enable_keyboard = true;
	ros::NodeHandle n;
    force_pub = n.advertise<geometry_msgs::Vector3>("control",1);
    enable_control_pub = n.advertise<std_msgs::Bool>("enable_control",1);
    keyboard_sub = n.subscribe("omniFirewire/keyboard/keydown", 1, &OmniController::keyboardPublisher, this);
    gamepad_sub = n.subscribe("omniFirewire/joy", 1, &OmniController::gamepadPublisher, this);
} 	

void OmniController::keyboardPublisher(const keyboard::Key::ConstPtr& msg)
{
	if (msg->code == keyboard::Key::KEY_TAB)
	{
		forceOutput.x = 0;
		forceOutput.y = 0;
		forceOutput.z = 0;
		enable_keyboard = true;
		enable_gamepad = false;
	}	
	
	if (!enable_keyboard)
	{
		return;
	}

	if (msg->code == keyboard::Key::KEY_r)
	{
		forceOutput.x = 0;
		forceOutput.y = 0;
		forceOutput.z = 0;	
		enable_control.data = true;
		enable_control_pub.publish(enable_control);
	}

	if (msg->code == keyboard::Key::KEY_d && this->forceOutput.x < 1.0)
	{
		forceOutput.x = this->forceOutput.x + 0.05;
	}

	if (msg->code == keyboard::Key::KEY_a && this->forceOutput.x > -1)
	{
		forceOutput.x = this->forceOutput.x - 0.05;
	}

	if (msg->code == keyboard::Key::KEY_w && this->forceOutput.y < 1)
	{
		forceOutput.y = this->forceOutput.y + 0.05;
	}

	if (msg->code == keyboard::Key::KEY_s && this->forceOutput.y > -1)
	{
		forceOutput.y = this->forceOutput.y - 0.05;
	}

	if (msg->code == keyboard::Key::KEY_e && this->forceOutput.z < 1)
	{
		forceOutput.z = this->forceOutput.z + 0.05;
	}

	if (msg->code == keyboard::Key::KEY_q && this->forceOutput.z > -1)
	{
		forceOutput.z = this->forceOutput.z - 0.05;
	}	

	force_pub.publish(forceOutput);
}

void OmniController::gamepadPublisher(const sensor_msgs::Joy::ConstPtr& msg)
{
	if (msg->buttons[6])			 
	{	
		forceOutput.x = 0;
		forceOutput.y = 0;
		forceOutput.z = 0;
		enable_gamepad = true;
		enable_keyboard = false;
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "omni_controller");
    OmniController omni;
    ros::spin();
    return 0;
}
