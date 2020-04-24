#include "../include/omnibase.h"

#include <Eigen/Geometry>

#include <iostream>


typedef boost::shared_ptr<urdf::Model> URDFModelPtr;
typedef boost::shared_ptr<srdf::Model> SRDFModelPtr;

OmniBase::OmniBase(const std::string &name, const std::string &path_urdf, const std::string &path_srdf)
    : OmniBase::OmniBase(name, path_urdf , path_srdf , 120)
{
}

OmniBase::OmniBase(const std::string &name, const std::string &path_urdf, const std::string &path_srdf, double vel_filter_wc)
    : force_feedback_gain(0),
    last_published_joint5_velocity(0),
    teleop_sensitivity(0),
    teleop_master(true),
    velocity_filter_wc(vel_filter_wc),
    name(name),
    enable_force_flag(false)
{
    node = ros::NodeHandlePtr( new ros::NodeHandle("") );

    // Prepare joint state publisher.
    topic_name = name + "joint_states";
    pub_joint = node->advertise<sensor_msgs::JointState>(topic_name, 10);

    // Prepare joint delta publisher.
    topic_name = name + "joint_delta";
    pub_delta = node->advertise<sensor_msgs::JointState>(topic_name, 10);

    // Prepare pose publisher.
    topic_name = name + "pose";
    pub_pose = node->advertise<geometry_msgs::PoseStamped>(topic_name, 10);
    pose_stamped.header.frame_id = name + "stylus";

    // Prepare pose publisher.
    topic_name = name + "twist";
    pub_twist = node->advertise<geometry_msgs::Twist>(topic_name, 10);

    // Prepare button state publisher and subscribe it.
    topic_name = name + "button_state";
    pub_button = node->advertise<omni_driver::OmniButtonEvent>(topic_name, 10);
    sub_button = node->subscribe(topic_name, 1, &OmniBase::buttonCallback, this);

    // Prepare teleop publisher
    topic_name = name + "teleop";
    pub_teleop_control = node->advertise<omni_driver::TeleopControl>(topic_name, 1);

    // Subscribe omni_control topic.
    topic_name = name + "control";
    sub_torque = node->subscribe(topic_name, 1, &OmniBase::torqueCallback, this);

    // Subscribe enable_control topic.
    topic_name = name + "enable_control";
    sub_enable_control = node->subscribe(topic_name, 1, &OmniBase::enableControlCallback, this);

    // Subscribe and initialize teleoperated joint states.
    topic_name = name + "teleop_joint_states";
    sub_slave_joint_states = node->subscribe(topic_name, 1, &OmniBase::teleopJointStatesCallback, this);
    teleoperated_joint_states.name.resize(6);
    teleoperated_joint_states.position.resize(6);
    teleoperated_joint_states.velocity.resize(6);
    joint_delta_ref.resize(6);
    teleoperated_joint_delta_ref.resize(6);


    // Subscribe to teleop topic if this omni is a slave
    ros::param::param<bool>("~teleop_master", teleop_master, true);
    if (!teleop_master)
        this->teleoperationSlave();
    else
        this->teleoperationForceFeedback();

    // Get the force feedback gain, twist gain, joint states gain and joint states offset
    ros::param::param<double>("~force_feedback_gain", force_feedback_gain, 1);
    ros::param::param<double>("~twist_gain", twist_gain, 1);
    ros::param::get("~joint_states_gain", joint_states_gain);
    if (joint_states_gain.size() != 6)
        throw std::logic_error("Joint states gain is represented as 6 elements vector.");
    ros::param::get("~joint_states_offsets", joint_states_offsets);
    if (joint_states_offsets.size() != 6)
        throw std::logic_error("Joint states offset is represented as 6 elements vector.");

    // Get the link names
    ros::param::param<std::string>("~link_base",         links.base.name, "base");
    ros::param::param<std::string>("~link_torso",        links.torso.name, "torso");
    ros::param::param<std::string>("~link_upper_arm",    links.upper_arm.name, "upper_arm");
    ros::param::param<std::string>("~link_lower_arm",    links.lower_arm.name, "lower_arm");
    ros::param::param<std::string>("~link_wrist",        links.wrist.name, "wrist");
    ros::param::param<std::string>("~link_tip",          links.tip.name, "tip");
    ros::param::param<std::string>("~link_stylus",       links.stylus.name, "stylus");
    ros::param::param<std::string>("~link_end_effector", links.end_effector.name, "end_effector");
    
    // Get link to teleop rotation
    std::vector<double> rot_data;
    ros::param::get("~rot_link_to_teleop_colwise", rot_data);
    if (rot_data.empty())
        rot_link_to_teleop.setIdentity();
    else if (rot_data.size() == 9)
        rot_link_to_teleop = Eigen::Matrix3d(rot_data.data());
    else
        throw std::logic_error("Rotation matrix is represented by a 9 element array");

    // Initialize teleop_control message fields
    teleop_control.vel_joint.resize(6);
    teleop_control.vel_effector.resize(6);

    std::memset(last_buttons, 0, sizeof(last_buttons));

    // Initializing robot_model for MoveIt!
    URDFModelPtr urdf_model = URDFModelPtr( new urdf::Model() );
    urdf_model->initFile(path_urdf);
    SRDFModelPtr srdf_model = SRDFModelPtr( new srdf::Model() );
    srdf_model->initFile(*urdf_model, path_srdf);

    kinematic_model = robot_model::RobotModelPtr( new robot_model::RobotModel(urdf_model, srdf_model) );

    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    joint_model_group = kinematic_model->getJointModelGroup("all");
    if (!joint_model_group)
    {
        // TODO - quit program
    }
    state.joint_names = joint_model_group->getJointModelNames();
    end_effector_link_model = kinematic_state->getLinkModel(links.end_effector.name);
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
    Eigen::VectorXd joint_angles(6);
    joint_angles << state.angles[0],
            state.angles[1],
            state.angles[2],
            state.angles[3],
            state.angles[4],
            state.angles[5];
    kinematic_state->setJointGroupPositions(joint_model_group, joint_angles);
    fwdKin();
//    calculateJointVelocities();   // No need to update the jointvelocity at high rate
//    std::vector<double> filtered_velocities;
//    filterVelocities(filtered_velocities);
//    calculateEffectorVelocities();
}


std::vector<double> OmniBase::calculateJointDeltas(std::vector<bool> button_state, sensor_msgs::JointState joint_state){
    if (button_state[0] || button_state[1]) {
        std::vector<double> joint_delta(6);
        /**
         * Commented for recursion because Tetis joint 4 is mapping Omni joint 5
         */
        // joint_delta.resize(6);
        // for (int i = 0; i < 6; ++i) {
        //     joint_delta[i] = (joint_state.position[i] - joint_delta_ref[i]) + teleoperated_joint_delta_ref[i];
        // }
        
        joint_delta[0] = joint_states_gain[0] * (joint_state.position[0] - joint_delta_ref[0]) + teleoperated_joint_delta_ref[0];
        joint_delta[1] = joint_states_gain[1] * (joint_state.position[1] - joint_delta_ref[1]) + teleoperated_joint_delta_ref[1];
        joint_delta[2] = joint_states_gain[2] * (joint_state.position[2] - joint_delta_ref[2]) + teleoperated_joint_delta_ref[2];
        joint_delta[3] = joint_states_gain[4] * (joint_state.position[4] - joint_delta_ref[4]) + teleoperated_joint_delta_ref[3];
        
        return joint_delta;
    }
    else {
        joint_delta_ref = joint_state.position;
        for (int i = 0; i < 4; ++i) {
            teleoperated_joint_delta_ref[i] = teleoperated_joint_states.position[i];
        }
        return teleoperated_joint_states.position;
    }
}

void OmniBase::fwdKin()
{
    Eigen::Affine3d end_effector_state;
    end_effector_state = kinematic_state->getGlobalLinkTransform(links.end_effector.name);
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

Eigen::Vector3d OmniBase::calculateTorqueFeedback(const Eigen::Vector3d& force, double feedback_gain)
{
    if (feedback_gain == 0)
        return Eigen::Vector3d::Zero();

    // Get the desired frame
    auto frame = links.lower_arm.name;

    // Get the Jacobian matrix written on the base frame
    Eigen::Vector3d origin(0,0.08,0);
    Eigen::MatrixXd jacobian;
    auto link_model = kinematic_state->getLinkModel(frame);
    kinematic_state->getJacobian(joint_model_group, link_model, origin, jacobian, false);

    // // Rotate the force vector from the desired frame to the base frame
    // Eigen::Vector3d force_on_link_frame = rot_link_to_teleop * force;
    // //
    // auto quat_base_link = kinematic_state->getGlobalLinkTransform(frame).rotation();
    // force_on_link_frame = quat_base_link.conjugate() * force_on_link_frame;

    Eigen::Vector3d force_on_link_frame = force;
    auto ret = feedback_gain * jacobian.block<3,3>(0,0).transpose() * force_on_link_frame;
    return ret.block<3,1>(0,0);
}

void OmniBase::calculateJointVelocities()
{
    double dt = 1/50.0;  // Now run at 50 hz

        for (int i = 0; i < 6; ++i)
        {
            state.vel_error[i] = state.angles[i] - state.vel_z[i];
 //           state.velocities[i] = velocity_filter_wc * state.vel_error[i];    // not sure where wc is set, so i hard coded  30 rad/s
 //           state.vel_z[i] += dt * velocity_filter_wc * state.vel_error[i];
            state.velocities[i] = 30.0 * state.vel_error[i];
            state.vel_z[i] += dt * 30.0 * state.vel_error[i];
        }

        Eigen::VectorXd joint_velocities(6);
        joint_velocities << state.velocities[0],
                state.velocities[1],
                state.velocities[2],
                state.velocities[3],
                state.velocities[4],
                state.velocities[5];
        kinematic_state->setJointGroupVelocities(joint_model_group,joint_velocities);
}

// void OmniBase::calculateVelocities()
// {


//     double deltaAngle;
//     TimeDuration time = state.time_current_angle_acquisition
//             - state.time_last_angle_acquisition;
//     double dt = time.total_microseconds();

//     if (dt >= velocity_filter_minimum_dt)
//     {
//         double tol_dvel = 3; // This is empirical. Deal with it.
//         for (int i = 0; i < 6; ++i)
//         {
//             deltaAngle = state.angles[i] - state.angles_hist1[i];

//             double vel = deltaAngle * 1000000 / dt;

//             if ( std::abs(vel - state.vel_hist1[i]) < tol_dvel )
//             {
//                 state.velocities[i] = vel;
//             }
//             state.vel_hist1[i] = vel;
//         }

//         state.angles_hist1 = state.angles;
//         state.time_last_angle_acquisition = state.time_current_angle_acquisition;
//         Eigen::VectorXd joint_velocities(6);
//         joint_velocities << state.velocities[0],
//                 state.velocities[1],
//                 state.velocities[2],
//                 state.velocities[3],
//                 state.velocities[4],
//                 state.velocities[5];
//         kinematic_state->setJointGroupVelocities(joint_model_group,joint_velocities);
//     }
// }

// void OmniBase::filterVelocities(std::vector<double> &filtered_velocities)
// {

//     if (velocities_filter_size <= 2)
//     {
//         velocities_filter.insert(velocities_filter.begin(), state.velocities);
//         velocities_filter.resize(velocities_filter_size);
//         double velocities_mean = 0;

//         for (unsigned int i =0; i < state.velocities.size(); i++)
//         {
//             for (unsigned int j =0; j < velocities_filter_size; j++)
//             {
//                 velocities_mean += velocities_filter[i][j];
//             }
//             velocities_mean = velocities_mean/velocities_filter_size;
//             filtered_velocities[i] = velocities_mean;
//         }
//         state.velocities = filtered_velocities;
//     }
// }

void OmniBase::calculateEffectorVelocities()
{
    Eigen::Vector3d origin(0,0,0);
    Eigen::MatrixXd jacobian(6,6);

    // To use the pen position  use links.end_effector.name   instead of wrist position   links.tip.name 
    auto current_end_effector_link_model = kinematic_state->getLinkModel(links.tip.name);  // Consider the wrist frame (intersection of joint 4-5-6)
    kinematic_state->getJacobian(joint_model_group, current_end_effector_link_model, origin, jacobian, false);

    Eigen::VectorXd thetaDot(6);

    thetaDot(0) = state.velocities[0];
    thetaDot(1) = state.velocities[1];
    thetaDot(2) = state.velocities[2];
    thetaDot(3) = state.velocities[3];
    thetaDot(4) = state.velocities[4];
    thetaDot(5) = state.velocities[5];

    Eigen::VectorXd xDot(6);

    xDot = jacobian * thetaDot;

    for (int i=0; i<6; ++i)
    {
        state.twist[i] = xDot(i);
    }
}

void OmniBase::teleoperationForceFeedback()
{
    sub_force = node->subscribe("force_feedback", 1, &OmniBase::forceFeedbackCallback, this);
}

void OmniBase::forceFeedbackCallback(const std_msgs::Float64MultiArray::ConstPtr& force)
{
    // Optoforce is not precise enough on all axis.
    Eigen::Vector3d force_vector(0, force->data[0], 0);
    Eigen::Vector3d joint_torques = OmniBase::calculateTorqueFeedback(force_vector, force_feedback_gain);
    std::vector<double> torque_input(3);
    std::copy(joint_torques.data(), joint_torques.data() + 3, torque_input.begin());
    this->setTorque(torque_input);
}

void OmniBase::teleoperationSlave()
{
    sub_teleop = node->subscribe("/teleop", 1, &OmniBase::jointTeleopCallback, this);
}

void OmniBase::jointTeleopCallback(const omni_driver::TeleopControl::ConstPtr& msg)
{
    std::vector<double> control;
    switch (msg->mode)
    {
    case 0: // joint space
        if (msg->vel_joint.size() < 3)
        {
            ROS_ERROR("TELEOP: Joint velocity vector should contain at least 3 elements!");
            return;
        }
        control = msg->vel_joint;
        control.resize(3);
        this->setTorque(control);
        break;
    case 1: // cartesian space
        std::cerr << "Control type 1" << std::endl;

        //todo
        break;
    }
}

void OmniBase::buttonCallback(const omni_driver::OmniButtonEvent::ConstPtr &msg)
{
    if (teleop_master)
    {
        // if (msg->grey_button_clicked)
        // {
        //     ++teleop_sensitivity;

        //     ROS_INFO_STREAM(teleop_sensitivity);
        // }
        // if (msg->white_button_clicked)
        // {
        //     if (teleop_sensitivity-- > 0);
        //     else
        //         teleop_sensitivity = 0;

        //     ROS_INFO_STREAM(teleop_sensitivity);
        // }
    }
    else
    {
        if (msg->grey_button_clicked)
        {
            enableControl(true);
            ROS_INFO("TELEOP: EnableControl = true");
        }
        if (msg->white_button_clicked)
        {
            enableControl(false);
            ROS_INFO("TELEOP: EnableControl = false");
        }
    }
}

void OmniBase::publishOmniState()
{
    if (!this->connected() /*|| !this->calibrated()*/)
    {
        // Phantom Omni is not open or calibrated. Don't publish.
        return;
    }

    // Get the joint angles from the omni.
    std::vector<double> joint_angles, joint_velocities;
    this->getJointAngles(joint_angles);
    this->getJointVelocities(joint_velocities);

    // Calculate velocity at 50 hz
    calculateJointVelocities();
    calculateEffectorVelocities();

    // Publish the joint state.
    joint_state.header.stamp = ros::Time::now();

    for (int i = 0; i < 6; ++i)
    {
        joint_state.position[i] = joint_angles[i] + joint_states_offsets[i];
        joint_state.velocity[i] = joint_states_gain[i] * joint_velocities[i];
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

    // Publish the twist.
    if (button_state[0] || button_state[1]) {
        twist.linear.x = twist_gain * state.twist[0];
        twist.linear.y = twist_gain * state.twist[1];
        twist.linear.z = twist_gain * state.twist[2];
        twist.angular.x = twist_gain * state.twist[3];
        twist.angular.y = twist_gain * state.twist[4];
        twist.angular.z = twist_gain * state.twist[5];    
    }
    else {
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;
    }
    pub_twist.publish(twist);

    // Publish the teleoperation data.
    if (teleop_master)
    {
        // Publish the joint delta.
        std_msgs::Float64MultiArray joint_delta;
        joint_delta.data = calculateJointDeltas(button_state, joint_state);
        sensor_msgs::JointState joint_state_delta;

        joint_state_delta.header.stamp = ros::Time::now();
        joint_state_delta.name.resize(6);
        joint_state_delta.position.resize(6);
        joint_state_delta.velocity.resize(6);
        for (int i = 0; i < 6; ++i) {
            joint_state_delta.position[i] = joint_delta.data[i];
        }
        pub_delta.publish(joint_state_delta);
        // Publish teleop data for other omni.
        const double sensitivity_step = 0.001;
        for (unsigned int k = 0; k < state.velocities.size(); ++k)
        {
            teleop_control.vel_joint[k] = teleop_sensitivity * sensitivity_step * state.velocities[k];
        }
        teleop_control.vel_effector = state.twist;
        teleop_control.mode = 0;
        pub_teleop_control.publish(teleop_control);
    }


    // Check if the device is frozen. This usually happens when the read buffer
    // is too small and it is completely filled (at least that is what we think).
    if (state.velocities[5] == last_published_joint5_velocity)
    {
        if (++state.freeze_count > MAX_FREEZE_COUNT)
        {
            state.freeze_count = 0;
            this->wakeup();
        }
    }
    else
    {
        state.freeze_count = 0;
    }
    last_published_joint5_velocity = state.velocities[5];
}

void OmniBase::torqueCallback(const geometry_msgs::Vector3::ConstPtr & msg)
{
    std::vector<double> input = {msg->x, msg->y, msg->z};
    setTorque(input);
}

void OmniBase::teleopJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for (int i = 0; i < 4; ++i) {
        teleoperated_joint_states.position[i] = msg->position[i];
    }
}
