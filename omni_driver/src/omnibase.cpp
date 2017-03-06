#include "../include/omnibase.h"

#include <Eigen/Geometry>

#include <iostream>

typedef boost::shared_ptr<urdf::Model> URDFModelPtr;
typedef boost::shared_ptr<srdf::Model> SRDFModelPtr;

OmniBase::OmniBase(const std::string &name)
        : name(name), enable_force_flag(false)
{
    this->resetTorque();

    node = ros::NodeHandlePtr( new ros::NodeHandle("") );

    // Prepare joint state publisher.
    topic_name = name + "joint_states";
    pub_joint = node->advertise<sensor_msgs::JointState>(topic_name, 10);

    // Prepare pose publisher.
    topic_name = name + "pose";
    pub_pose = node->advertise<geometry_msgs::PoseStamped>(topic_name, 10);
    pose_stamped.header.frame_id = name + "stylus";

    // Prepare button state publisher.
    topic_name = name + "button_state";
    pub_button = node->advertise<omni_driver::OmniButtonEvent>(topic_name, 10);

    // Subscribe omni_control topic.
    topic_name = name + "control";
    sub_torque = node->subscribe(topic_name, 1, &OmniBase::torqueCallback, this);

    // Subscribe enable_control topic.
    topic_name = name + "enable_control";
    sub_enable_control = node->subscribe(topic_name, 1, &OmniBase::enableControlCallback, this);

    // Subscribe button_state topic.
    topic_name = name + "button_state";

    std::memset(last_buttons, 0, sizeof(last_buttons));

    // Initializing robot_model for MoveIt!
    std::string path_urdf = "/home/brunogbv/omni_ws/src/PhantomOmni/omni_description/urdf/omni.urdf";
    std::string path_srdf = "/home/brunogbv/omni_ws/src/omni_moveit/config/phantom_omni.srdf";

    URDFModelPtr urdf_model = URDFModelPtr( new urdf::Model() );
    urdf_model->initFile(path_urdf);
    SRDFModelPtr srdf_model = SRDFModelPtr( new srdf::Model() );
    srdf_model->initFile(*urdf_model, path_srdf);

    kinematic_model = robot_model::RobotModelPtr( new robot_model::RobotModel(urdf_model, srdf_model) );

    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    joint_model_group = kinematic_model->getJointModelGroup("all");
    state.joint_names = joint_model_group->getJointModelNames();
    //
    const int n = state.joint_names.size();
    joint_state.name.resize(n);
    joint_state.position.resize(n);
    joint_state.velocity.resize(n);
    for (int k = 0; k < n; ++k)
    {
        joint_state.name[k] = name + state.joint_names[k];
    }
}


void OmniBase::enableControlCallback(const std_msgs::Bool::ConstPtr & msg)
{
    enableControl(msg->data);
}

void OmniBase::updateRobotState()
{
    fwdKin();
    calculateVelocities();
}

void OmniBase::fwdKin(const unsigned int idx)
{
    Eigen::Affine3d end_effector_state;
    if (idx >= state.joint_names.size())
    {
        ROS_ERROR("Index exceeds the number of joints.");
        return;
    }

    end_effector_state = kinematic_state->getJointTransform(state.joint_names[idx]);
    Eigen::Quaterniond quat(end_effector_state.rotation());
    Eigen::Vector3d pos = end_effector_state.translation();

    state.orientation[0] = quat.w();
    state.orientation[1] = quat.x();
    state.orientation[2] = quat.y();
    state.orientation[3] = quat.z();

    state.position[0] = pos[0];
    state.position[1] = pos[1];
    state.position[2] = pos[2];
}

void OmniBase::calculateVelocities()
{
    TimeDuration time = state.time_current_angle_acquisition
            - state.time_last_angle_acquisition;
    double time_difference = (time.total_microseconds());

    for (int i=0; i<6; ++i)
    {
        state.velocities[i] = (state.angles[i] - state.angles_hist1[i]) * 1000000 / (time_difference);

    }

    state.angles_hist1 = state.angles;
}


void OmniBase::publishOmniState()
{
    if (!this->connected() || !this->calibrated()) {
        // Phantom Omni is not open or calibrated. Don't publish.
        return;
    }

    // Get the joint angles from the omni.
    std::vector<double> joint_angles, joint_velocities;
    this->getJointAngles(joint_angles);
    this->getJointVelocities(joint_velocities);

    // Publish the joint state.
    joint_state.header.stamp = ros::Time::now();

    for (int i = 0; i < 6; ++i)
    {
        joint_state.position[i] = joint_angles[i];
        joint_state.velocity[i] = joint_velocities[i];

        if (i==4)
        {
            // The next line is a necessary to recalibrate potentiometer #2.
            joint_state.position[4]=(joint_state.position[4]+0.27+(joint_state.position[4]+1.73)/1.72);
        }


    }

    // Publish the joint state;
    pub_joint.publish(joint_state);

    // Publish the end effector pose.
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.pose.position.x = state.position[0];
    pose_stamped.pose.position.y = state.position[1];
    pose_stamped.pose.position.z = state.position[2];
    pose_stamped.pose.orientation.w = state.orientation[0];
    pose_stamped.pose.orientation.x = state.orientation[1];
    pose_stamped.pose.orientation.y = state.orientation[2];
    pose_stamped.pose.orientation.z = state.orientation[3];
    pub_pose.publish(pose_stamped);

    // Publish the button event.
    std::vector<bool> button_state;
    this->getButtonsState(button_state);
    button_event.grey_button = button_state[0];
    button_event.white_button = button_state[1];
    button_event.grey_button_clicked = !last_buttons[0] && button_state[0];
    button_event.white_button_clicked = !last_buttons[1] && button_state[1];
    last_buttons[0] = button_state[0];
    last_buttons[1] = button_state[1];
    pub_button.publish(button_event);
}

void OmniBase::torqueCallback(const geometry_msgs::Vector3::ConstPtr & msg)
{
    std::vector<double> input = {msg->x, msg->y, msg->z};
    setTorque(input);
}
