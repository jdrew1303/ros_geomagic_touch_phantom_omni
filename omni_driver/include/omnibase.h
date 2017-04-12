#pragma once

#include <vector>

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "omni_driver/OmniButtonEvent.h"
#include "omni_driver/TeleopControl.h"

class OmniBase
{
private:
    double last_published_joint5_velocity;

    unsigned int teleop_sensitivity;

    bool teleop_master;

    static const unsigned int VELOCITIES_FILTER_SIZE = 5;

    int vel_filter_counter;

    std::vector<std::vector<double>> velocities_filter;

    static const unsigned int MAX_FREEZE_COUNT = 100;

    boost::shared_mutex mutex_state;                        ///< Mutex state returned by @link getMutexState().

    template <typename Type>
    void checkLimits(Type & value, Type min, Type max)
    {
        if (value < min)
        {
            value = min;
        }
        else if (value > max)
        {
            value = max;
        }
    }

    template <typename Type>
    void checkLimits(std::vector<Type> & values, Type min, Type max)
    {
        for (auto iter = values.begin(); iter < values.end(); ++iter)
        {
            checkLimits(*iter, min, max);
        }
    }

    void fwdKin(const unsigned int idx = 5);

    void calculateVelocities();

    void filterVelocities(std::vector<double> &filtered_velocities);

    void getEffectorVelocity();

protected:
    typedef boost::posix_time::microsec_clock Clock;
    typedef boost::posix_time::ptime Time;
    typedef boost::posix_time::time_duration TimeDuration;

    struct OmniState                                        ///< The state structure with relevant data.
    {
        std::vector<double> angles;
        std::vector<double> angles_docked;
        std::vector<double> angles_hist1;
        std::vector<double> angles_last;
        std::vector<double> control;
        std::vector<double> force;
        std::vector<double> lock_pos;
        std::vector<double> orientation;
        std::vector<double> position;
        std::vector<double> velocities;
        std::vector<double> vel_hist1;
        std::vector<double> twist;
        std::vector<bool>   buttons;
        std::vector<std::string> joint_names;
        Time time_last_angle_acquisition;
        Time time_current_angle_acquisition;
        bool control_on = false;
        bool connected = false;
        bool calibrated = false;
        bool lock = false;
        unsigned int seq = 0;
        Time stamp;
        unsigned int freeze_count;

        OmniState()
        {
            // Joints
            angles.resize(6);
            angles_docked.resize(6);
            velocities.resize(6);
            joint_names.resize(6);
            angles_hist1.resize(6);
            vel_hist1.resize(6);
            twist.resize(6);

            // Buttons
            buttons.resize(2);

            // Others
            control.resize(3);
            force.resize(3);
            position.resize(3);
            orientation.resize(4);
        }
    };

    OmniState state;

    std::string name;                           ///< Name given to diferentiate multiple omnis.
    std::string topic_name;                     ///< String used to subscribe diferent topics.

    ros::NodeHandlePtr node;
    ros::Subscriber sub_torque;                 ///< Torque ROS subscriber.
    ros::Subscriber sub_enable_control;         ///< Enable control ROS subscriber.
    ros::Subscriber sub_haptic;                 ///< Enable haptic ROS subscriber.
    ros::Subscriber sub_moveit;                 ///< Enable button subscriber for moveit.

    ros::Publisher pub_joint;                   ///< Joint ROS publisher.
    sensor_msgs::JointState joint_state;

    ros::Publisher pub_pose;                    ///< Pose ROS publisher.
    geometry_msgs::PoseStamped pose_stamped;

    ros::Publisher pub_twist;
    geometry_msgs::Twist twist;

    ros::Publisher pub_button;                  ///< Button ROS publisher.
    ros::Subscriber sub_button;

    ros::Subscriber sub_teleop;
    ros::Publisher pub_teleop_control;

    omni_driver::OmniButtonEvent button_event;
    omni_driver::TeleopControl teleop_control;

    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;
    robot_state::JointModelGroup* joint_model_group;
    const robot_state::LinkModel* end_effector_link_model;


    bool last_buttons[2];                       ///< Needed for "Button Clicked" logic.
    bool enable_force_flag;                     ///< Needed for resetting the internal enable control.
    double velocity_filter_minimum_dt;          ///< TODO value in milliseconds

    typedef boost::unique_lock<boost::shared_mutex>            LockUnique;          ///< The unique lock, used to protect data while writing.
    typedef boost::shared_lock<boost::shared_mutex>            LockShared;          ///< The shared lock, used to protect data while reading.
    typedef boost::upgrade_lock<boost::shared_mutex>           LockUpgrade;         ///< The upgradeable lock, used to protect data while reading.
    typedef boost::upgrade_to_unique_lock<boost::shared_mutex> LockUpgradeToUnique; ///< The upgraded lock, used to protect data while writing.

    /**
     * @brief Gets the mutex used for accessing the robot's state.
     * @return The mutex.
     */
    inline boost::shared_mutex & getStateMutex()
    {
        return mutex_state;
    }

    /**
     * @brief Gets the robot's state.
     * @return The robot's state.
     */
    inline OmniState * getState()
    {
        return &state;
    }

    /**
     * @brief Called by the base after a new control value has been received.
     * @see setTorque
     */
    virtual void mapTorque() = 0;


public:

    /**
     * @brief OmniBase constructor, sets some members and prepares ros topics and publishers.
     * @param name Reference to string of omni name.
     */
    explicit OmniBase(const std::string &name = "");

    /**
     * @brief OmniBase constructor, sets some members and prepares ros topics and publishers.
     * @param name Reference to string of omni name.
     * @param velocity_filter_minimum_dt Minimum amount of time that should have passed to compute the robot velocity.
     */
    OmniBase(const std::string &name, double velocity_filter_minimum_dt);

    /**
     * @brief Publishes the current robot's state.
     */
    void publishOmniState();

    /**
     * @brief Attempts to connect to the robot.
     * @return True if the connection succeeds. False otherwise.
     */
    virtual bool connect() = 0;

    /**
     * @brief Closes robot connection.
     */
    virtual void disconnect() = 0;

    /**
     * @brief Checks if the device is connected.
     * @return True if connected. False otherwise.
     * @see connect, disconnect
     */
    virtual bool connected() = 0;

    /**
     * @brief Wakes up the communications if it is frozen.
     */
    virtual void wakeup() = 0;

    /**
     * @brief Gets the current joint angles.
     * @param angles std::Vector that will store the angles.
     */
    inline void getJointAngles(std::vector<double> &angles)
    {
        // Protect the critical section.
        LockShared lock( getStateMutex() );
        angles = state.angles;
    }

    /**
     * @brief Gets the current joint angles.
     * @param angles Eigen::VectorXd that will store the angles.
     */
    inline void getJointAngles(Eigen::VectorXd angles)
    {
        // Protect the critical section.
        LockShared lock( getStateMutex() );
        angles << state.angles[0],
                        state.angles[1],
                        state.angles[2],
                        state.angles[3],
                        state.angles[4],
                        state.angles[5];
    }

    /**
     * @brief Gets the current buttons' state.
     * @param button std::Vector that will store the states.
     */
    void getButtonsState(std::vector<bool>& button)
    {
        // Protect the critical section.
        LockShared lock( getStateMutex() );
        button = state.buttons;
    }

    /**
     * @brief Gets the current force acting on the tip.
     * @param force std::vector that will store the force.
     */
    inline void getForce(std::vector<double> &force)
    {
        LockShared lock( getStateMutex() );
        force = state.force;
    }

    /**
     * @brief Sets the torque on the first three joints.
     * @param torque 3-elements std::vector with the torque values.
     */
    inline void setTorque(std::vector<double> &torque)
    {
        LockUnique lock( getStateMutex() );
        checkLimits(torque, -1.0, 1.0);
        state.control = torque;
        this->mapTorque();
    }

    /**
     * @brief Resets the torque on the first three joints.
     * @see setTorque
     */
    inline void resetTorque()
    {
        std::vector<double> null_torque(3,0);
        this->setTorque(null_torque);
    }

    /**
     * @brief Chooses if the control on the first three joints is on.
     * @param enable True to enable. False otherwise.
     */
    virtual void enableControl(bool enable) = 0;

    /**
     * @brief Gets the current joint velocities.
     * @param vel std::vector that will store the velocities.
     */
    inline void getJointVelocities(std::vector<double> &vel)
    {
        LockShared lock( getStateMutex() );
        vel = state.velocities;
    }

    /**
     * @brief Gets the current joint velocities.
     * @param vel Eigen::VectorXd that will store the velocities.
     */
    inline void getJointVelocities(Eigen::VectorXd vel)
    {
        LockShared lock( getStateMutex() );
        vel << state.velocities[0],
               state.velocities[1],
               state.velocities[2],
               state.velocities[3],
               state.velocities[4],
               state.velocities[5];
    }

    /**
     * @brief Gets the current tip position with respect to the robot's base frame.
     * @param pos std::vector that will store the position.
     */
    inline void getTipPosition(std::vector<double> &pos)
    {
        LockShared lock( getStateMutex() );
        pos = state.position;
    }

    /**
     * @brief Gets the current tip orientation with respect to the robot's base frame.
     * @param ori Vector that will store the orientation as a quaternion.
     */
    inline void getTipOrientation(std::vector<double> &ori)
    {
        LockShared lock( getStateMutex() );
        ori = state.orientation;
    }

    /**
     * @brief Gets the current tip position and orientation with respect to the robot's base frame.
     * @param pos Vector that will store the position.
     * @param ori Vector that will store the orientation as a quaternion.
     * @see getTipPosition, getTipOrientation
     */
    inline void getTipPose(std::vector<double> &pos, std::vector<double> &ori)
    {
        LockShared lock( getStateMutex() );
        pos = state.position;
        ori = state.orientation;
    }

    /**
     * @brief Gets the omni name param.
     * @return Returns the robot's name.
     */
    inline std::string getTopicName()
    {
        return this->name;
    }

    /**
     * @brief Checks if the device has been calibrated.
     * @return True if calibrated. False otherwise.
     */
    inline bool calibrated()
    {
        LockShared lock( getStateMutex() );
        return state.calibrated;
    }

    void updateRobotState();

public:

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

    void teleoperationSlave();

    void jointTeleopCallback(const omni_driver::TeleopControl::ConstPtr& msg);

    void buttonCallback(const omni_driver::OmniButtonEvent::ConstPtr& msg);


};

typedef boost::shared_ptr<OmniBase> OmniBasePtr;
