#pragma once

#include "include/driver/omni_driver.hpp"
#include "include/util/typedefs.hpp"
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include "omni_driver/OmniButtonEvent.h"
#include "omni_driver/TeleopControl.h"
#include <vector>

class OmniRosInterface {
    private: 

    OmniDriverPtr omni_driver;

    Eigen::Matrix3d rot_link_to_teleop;

    double force_feedback_gain;

    std::vector<double> joint_delta_ref(6);

    std::vector<double> joint_states_offsets;

    std::vector<double> joint_states_gain;

    double twist_gain;

    bool teleop_master;

    std::string name;                           ///< Name given to diferentiate multiple omnis.

    bool last_buttons[2];                       ///< Needed for "Button Clicked" logic.

    std::string topic_name;                     ///< String used to subscribe diferent topics.\
    ros::NodeHandlePtr node;
    ros::Subscriber sub_torque;                 ///< Torque ROS subscriber.
    ros::Subscriber sub_enable_control;         ///< Enable control ROS subscriber.
    ros::Subscriber sub_haptic;                 ///< Enable haptic ROS subscriber.
    ros::Subscriber sub_moveit;                 ///< Enable button subscriber for moveit.

    ros::Publisher pub_joint;                   ///< Joint ROS publisher.
    ros::Publisher pub_delta;                   ///< Joint deltas ROS publisher.
    sensor_msgs::JointState joint_state;

    ros::Publisher pub_pose;                    ///< Pose ROS publisher.
    geometry_msgs::PoseStamped pose_stamped;

    ros::Publisher pub_twist;
    geometry_msgs::Twist twist;

    ros::Publisher pub_button;                  ///< Button ROS publisher.
    ros::Subscriber sub_button;

    ros::Subscriber sub_teleop;
    ros::Subscriber sub_force;
    ros::Publisher pub_teleop_control;

    omni_driver::OmniButtonEvent button_event;
    omni_driver::TeleopControl teleop_control;

    /**
     * @brief The torqueCallback called by the ROS torque subscriber.
     * @param ROS message type geometry_msgs::Vector3.
     */
    void torqueCallback(const geometry_msgs::Vector3::ConstPtr& msg);

    /**
     * @brief The enableControlCallback called by the ROS enable control subscriber.
     * @param ROS message type geometry_msgs::Bool.
     */
    void enableControlCallback(const std_msgs::Bool::ConstPtr& msg);

    /**
     * @brief teleoperation Method used to teleoperate other robots using the omni device. To use it, pass the desired teleoperation robot name.
     * Note that the robot must have the default topic names, i.e. robot_name/joint_states, robot_name/twist, etc.
     * @param robot_name String correspondig to your robot prefix in the ROS topics.
     */
    void teleoperationMaster(std::string robot_name);

    void teleoperationForceFeedback();

    void forceFeedbackCallback(const std_msgs::Float64MultiArray::ConstPtr& force);

    void teleoperationSlave();

    void jointTeleopCallback(const omni_driver::TeleopControl::ConstPtr& msg);

    void buttonCallback(const omni_driver::OmniButtonEvent::ConstPtr& msg);

    public: 

    void connect() = omni_driver.connect();
    
    void disconnect() = omni_driver.disconnect();
    
    void connect() = omni_driver.connect();
    
    void connected() = omni_driver.connected();

    void publishOmniState();

    OmniRosInterface(
        OmniDriverPtr omni_driver;
        Eigen::Matrix3d rot_link_to_teleop;
        double force_feedback_gain;
        std::vector<double> joint_states_offsets;
        std::vector<double> joint_states_gain;
        double twist_gain;
        bool teleop_master;
        std::string name;                           ///< Name given to diferentiate multiple omnis.
    );
    ~OmniRosInterface();
};
