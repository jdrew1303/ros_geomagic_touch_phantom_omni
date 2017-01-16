/*
 * Copyright (c) 2013, 2015 Yutaka Tsutano.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
 * REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT,
 * INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
 * LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
 * OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

#include <vector>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <raw_omni/OmniButtonEvent.h>
#include <std_msgs/Bool.h>

#include "raw_omni_driver.h"
#include "node/Omni.h"

class RawOmniNode {
private:
    std::string name_;

    RawOmniDriver driver_;

    ros::Publisher joint_pub_;
    sensor_msgs::JointState joint_state_;

    ros::Publisher pose_pub_;
    geometry_msgs::PoseStamped pose_stamped_;

    ros::Publisher button_pub_;
    raw_omni::OmniButtonEvent button_event_;

    ros::Subscriber force_sub_;

    ros::Subscriber enable_control_sub_;

    ros::Timer timer_;
    
    bool last_buttons[2];

public:
    RawOmniNode(const std::string& name, const std::string& serial)
            : name_(name), driver_(serial)
    {
        ros::NodeHandle n;

        // Prepare joint state publisher.
        {
            std::ostringstream topic_name;
            topic_name << name_ << "joint_states";
            joint_pub_ = n.advertise<sensor_msgs::JointState>(
                    topic_name.str(), 10);

            joint_state_.name.resize(6);
            joint_state_.position.resize(6);
            joint_state_.name[0] = name_ + "waist";
            joint_state_.name[1] = name_ + "shoulder";
            joint_state_.name[2] = name_ + "elbow";
            joint_state_.name[3] = name_ + "wrist1";
            joint_state_.name[4] = name_ + "wrist2";
            joint_state_.name[5] = name_ + "wrist3";
        }

        // Prepare pose publisher.
        {
            std::ostringstream topic_name;
            topic_name << name_ << "pose";
            pose_pub_ = n.advertise<geometry_msgs::PoseStamped>(
                    topic_name.str(), 10);

            pose_stamped_.header.frame_id = name_ + "stylus";
        }
        
        // Prepare button state isher
        {
            std::ostringstream topic_name;
            topic_name << name_ << "button_state";
            button_pub_ = n.advertise<raw_omni::OmniButtonEvent>(
                    topic_name.str(), 10);            
        }

       // Subscribe omni_control topic
        {
            std::ostringstream topic_name;
            topic_name << name_ << "control";
            force_sub_ = n.subscribe(topic_name.str(), 1, &RawOmniNode::forceCallback, this); 
        }
        
		// Subscribe enable_control topic
        {
            std::ostringstream topic_name;
            topic_name << name_ << "enable_control";
            enable_control_sub_ = n.subscribe(topic_name.str(), 1, &RawOmniNode::enableCallback, this); 
        }

        // Create timer for communication.
        timer_ = n.createTimer(ros::Duration(0.0025),
                &RawOmniNode::timerHandler, this);
                
        std::memset(last_buttons, 0, sizeof(last_buttons));
    }

    void forceCallback(const geometry_msgs::Vector3::ConstPtr& msg)
	{
		const double force [] = {msg->x, -msg->y, msg->z};
		driver_.set_force(force);
	}

	void enableCallback(const std_msgs::Bool::ConstPtr& msg)
	{
		driver_.enable_force(msg->data);
	}
    
    std::string get_topic_name()
    {
    	return this->name_;
    }
    ~RawOmniNode()
    {
        driver_.close();
    }

    void timerHandler(const ros::TimerEvent& event)
    {
        if (driver_.opened() && !driver_.calibrated()) {
            // Phantom Omni is not open or calibrated. Don't publish.
            return;
        }

        // Get the joint angles from the omni.
        std::vector<double> joint_angles;
        driver_.get_current_joint_angles(joint_angles);

        // Publish the joint state.
        joint_state_.header.stamp = ros::Time::now();
        for (int i = 0; i < 6; i++) {
            joint_state_.position[i] = joint_angles[i];
        }
        //the next line is a necessary to recalibrate potentiometer #2
        joint_state_.position[4]=(joint_state_.position[4]+0.27+(joint_state_.position[4]+1.73)/1.72);

        joint_pub_.publish(joint_state_);

        // Publish the end effector pose.
        pose_stamped_.header.stamp = ros::Time::now();
        pose_stamped_.pose.position.x = 0.0;
        pose_stamped_.pose.orientation.w = 1.0;
        pose_pub_.publish(pose_stamped_);
    
        // Publish the button state.
        std::vector<bool> button_state;
        driver_.get_current_button_state(button_state);
        button_event_.grey_button = button_state[0];
        button_event_.white_button = button_state[1];
        //
        button_event_.grey_button_clicked = !last_buttons[0] && button_state[0];
        button_event_.white_button_clicked = !last_buttons[1] && button_state[1];
        last_buttons[0] = button_state[0];
        last_buttons[1] = button_state[1];
        //
        button_pub_.publish(button_event_);
    }

    bool run()
    {
        // Open Phantom Omni.
        if (!driver_.open()) {
            ROS_ERROR("Failed to open Phantom Omni.");
            return false;
        }
        ROS_INFO("%s (%s): opened.",
                name_.c_str(), driver_.serial_number().c_str());

        // Spin.
        ros::spin();

        return true;
    }
};

/**
 * Entry point.
 */
int main(int argc, char** argv)
{
    // Initialize ROS.
    ros::init(argc, argv, "raw_omni");

    // Get the omni parameters.
    std::string omni_name;
    ros::param::param<std::string>("~omni_name", omni_name, "");
    std::string omni_serial;
    ros::param::param<std::string>("~omni_serial", omni_serial, "11129400000");

    // List the serial number of each Phantom Omni connected.
    const std::vector<RawOmniDriver::OmniInfo>& omnis = RawOmniDriver::enumerate_omnis();
    std::ostringstream serial_numbers;
    for (size_t i = 0; i < omnis.size(); i++) {
        serial_numbers << omnis[i].serial << " ";
    }
    ROS_INFO("Phantom Omni Serial Numbers: %s", serial_numbers.str().c_str());

    // Start the node.
    RawOmniNode node(omni_name, omni_serial);
    node.run();

    return 0;
}
