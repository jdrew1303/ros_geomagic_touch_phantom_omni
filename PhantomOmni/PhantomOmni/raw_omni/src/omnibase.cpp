#include "../include/omnibase.h"

#include <iostream>


OmniBase::OmniBase(const std::string& name, const std::string& serial)
    : name_(name), driver_(serial)
{
    node = ros::NodeHandlePtr( new ros::NodeHandle("~") );

    // Prepare joint state publisher.
    topic_name = name_ + "joint_states";
    joint_pub_ = node->advertise<sensor_msgs::JointState>(topic_name, 10);
    joint_state_.name.resize(6);
    joint_state_.position.resize(6);
    joint_state_.name[0] = name_ + "waist";
    joint_state_.name[1] = name_ + "shoulder";
    joint_state_.name[2] = name_ + "elbow";
    joint_state_.name[3] = name_ + "wrist1";
    joint_state_.name[4] = name_ + "wrist2";
    joint_state_.name[5] = name_ + "wrist3";

    // Prepare pose publisher.
    topic_name = name_ + "pose";
    pose_pub_ = node->advertise<geometry_msgs::PoseStamped>(topic_name, 10);
    pose_stamped_.header.frame_id = name_ + "stylus";

    // Prepare button state publisher.
    topic_name = name_ + "button_state";
    button_pub_ = node->advertise<raw_omni::OmniButtonEvent>(topic_name, 10);

    // Subscribe omni_control topic.
    topic_name = name_ + "control";
    torque_sub = node->subscribe(topic_name, 1, &OmniBase::torqueCallback, this);

    // Subscribe enable_control topic.
    topic_name << name_ << "enable_control";
    enable_control_sub_ = node->subscribe(topic_name, 1, &OmniBase::enableCallback, this);

    // Create timer for communication.
    timer_ = node->createTimer(ros::Duration(0.0025),&RawOmniNode::timerHandler, this);


    std::memset(last_buttons, 0, sizeof(last_buttons));


}

void OmniBase::torqueCallback(const geometry_msgs::Vector3_::ConstPtr& msg)
{
    std::vector<double> input = {msg->x, msg->y, msg->z};
    setTorque(input);
}

void OmniBase::enableControlCallback(const std_msgs::Bool_::ConstPtr &msg)
{
    enableControl(msg->data);
}

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
}

void OmniBase::driverCallbackRead(void *data)
{

}

void OmniBase::driverCallbackWrite(void *data)
{

}
