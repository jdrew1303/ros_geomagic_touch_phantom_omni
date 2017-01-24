#include "../include/omnibase.h"

#include <iostream>



void OmniBase::torqueCallback(const geometry_msgs::Vector3::ConstPtr & msg)
{
    std::vector<double> input = {msg->x, msg->y, msg->z};
    setTorque(input);
}

void OmniBase::enableControlCallback(const std_msgs::Bool::ConstPtr & msg)
{
    enableControl(msg->data);
}

/*
void OmniBase::driverCallback(void *data)
{
    OmniState * state = static_cast<OmniState *>(data);
    if (!state)
    {
        std::cerr << "Could not cast the data passed from the driver. The robot will not work." << std::endl;
        return;
    }

    // Acquire a unique lock to prevent others from writing
    LockUnique lock( getStateMutex() );

    // Call the implementation specific callback
    callback(state);

    // Store the current time
    Time time_now( Clock::local_time() );

    // If this is not the first callback call, update the velocity
    if (state->seq > 0)
    {
        TimeDuration dt_boost = time_now - state->stamp;
        double dt = ( (double) dt_boost.total_microseconds() ) / 1e+6;
        for (int k = 0; k < state->angles.size(); ++k)
        {
            state->velocities[k] = (state->angles[k] - state->angles_last[k]) / dt;
        }
    }

    // Compute the forward kinematics - TODO

    // Prepare for the next call
    state->angles_last = state->angles;
    state->stamp = time_now;
    state->seq++;
}*/

OmniBase::OmniBase(const std::string& name)
        : name(name)
{
    node = ros::NodeHandlePtr( new ros::NodeHandle("~") );

    // Prepare joint state publisher.
    topic_name = name + "joint_states";
    pub_joint = node->advertise<sensor_msgs::JointState>(topic_name, 10);
    joint_state.name.resize(6);
    joint_state.position.resize(6);
    joint_state.name[0] = name + "waist";
    joint_state.name[1] = name + "shoulder";
    joint_state.name[2] = name + "elbow";
    joint_state.name[3] = name + "wrist1";
    joint_state.name[4] = name + "wrist2";
    joint_state.name[5] = name + "wrist3";

    // Prepare pose publisher.
    topic_name = name + "pose";
    pub_pose = node->advertise<geometry_msgs::PoseStamped>(topic_name, 10);
    pose_stamped.header.frame_id = name + "stylus";

    // Prepare button state publisher.
    topic_name = name + "button_state";
    pub_button = node->advertise<raw_omni::OmniButtonEvent>(topic_name, 10);

    // Subscribe omni_control topic.
    topic_name = name + "control";
    sub_torque = node->subscribe(topic_name, 1, &OmniBase::torqueCallback, this);

    // Subscribe enable_control topic.
    topic_name = name + "enable_control";
    sub_enable_control = node->subscribe(topic_name, 1, &OmniBase::enableControlCallback, this);

    // Create timer for communication.
    timer = node->createTimer(ros::Duration(0.01), &OmniBase::timerCallback, this);

    std::memset(last_buttons, 0, sizeof(last_buttons));
}

void OmniBase::timerCallback(const ros::TimerEvent&)
{
    if (this->connected() && !this->calibrated()) {
        // Phantom Omni is not open or calibrated. Don't publish.
        return;
    }

    // Get the joint angles from the omni.
    std::vector<double> joint_angles;
    this->getJointAngles(joint_angles);

    // Publish the joint state.
    joint_state.header.stamp = ros::Time::now();
    for (int i = 0; i < 6; i++)
    {
        joint_state.position[i] = joint_angles[i];
    }
    //the next line is a necessary to recalibrate potentiometer #2
    joint_state.position[4]=(joint_state.position[4]+0.27+(joint_state.position[4]+1.73)/1.72);

    pub_joint.publish(joint_state);

    // Publish the end effector pose.
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.pose.position.x = 0.0;
    pose_stamped.pose.orientation.w = 1.0;
    pub_pose.publish(pose_stamped);

    // Publish the button state.
    std::vector<bool> button_state;
    this->getButtonsState(button_state);
    button_event.grey_button = button_state[0];
    button_event.white_button = button_state[1];
    //
    button_event.grey_button_clicked = !last_buttons[0] && button_state[0];
    button_event.white_button_clicked = !last_buttons[1] && button_state[1];
    last_buttons[0] = button_state[0];
    last_buttons[1] = button_state[1];
    //
    pub_button.publish(button_event);
}
