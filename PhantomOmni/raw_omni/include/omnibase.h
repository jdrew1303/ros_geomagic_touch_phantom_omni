#pragma once


#include <vector>

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <raw_omni/OmniButtonEvent.h>

class OmniBase
{
private:
    boost::shared_mutex mutex_state;

    void timerCallback(const ros::TimerEvent& event);

protected:
    typedef boost::posix_time::microsec_clock Clock;
    typedef boost::posix_time::ptime Time;
    typedef boost::posix_time::time_duration TimeDuration;

    struct OmniState
    {
        std::vector<double> angles_docked;
        std::vector<double> angles;
        std::vector<double> angles_last;
        std::vector<bool>   buttons;
        std::vector<double> control;
        std::vector<double> force;
        std::vector<double> position;
        std::vector<double> orientation;
        std::vector<double> velocities;

        bool control_on = false;
        bool calibrated = false;

        unsigned int seq = 0;
        Time stamp;
    };

protected:
    OmniState state;

    std::string name;
    std::string topic_name;

    ros::NodeHandlePtr node;

    ros::Subscriber sub_torque;

    ros::Publisher pub_joint;
    sensor_msgs::JointState joint_state;

    ros::Publisher pub_pose;
    geometry_msgs::PoseStamped pose_stamped;

    ros::Publisher pub_button;
    raw_omni::OmniButtonEvent button_event;

    ros::Subscriber sub_enable_control;

    ros::Timer timer;

    bool last_buttons[2];

protected:
    typedef boost::unique_lock<boost::shared_mutex>            LockUnique;
    typedef boost::shared_lock<boost::shared_mutex>            LockShared;
    typedef boost::upgrade_lock<boost::shared_mutex>           LockUpgrade;
    typedef boost::upgrade_to_unique_lock<boost::shared_mutex> LockUpgradeToUnique;

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

public:
    OmniBase(const std::string &name);

    /**
     * @brief Attempts to connect to the robot.
     * @return True if the connection succeeds. False otherwise.
     */
    virtual bool connect() = 0;

    /**
     * @brief Attempts to close the connection to the robot.
     * @return True if the connection was closed. False otherwise.
     */
    virtual void disconnect() = 0;

    /**
     * @brief Checks if the device is connected.
     * @return True if connected. False otherwise.
     * @see connect, disconnect
     */
    virtual bool connected() = 0;

    /**
     * @brief Gets the current joint angles.
     * @param angles Vector that will store the angles.
     */
    inline void getJointAngles(std::vector<double> &angles)
    {
        // Protect the critical section.
        LockShared lock( getStateMutex() );
        angles = state.angles;
    }

    /**
     * @brief Gets the current buttons' state.
     * @param button Vector that will store the states.
     */
    void getButtonsState(std::vector<bool>& button)
    {
        // Protect the critical section.
        LockShared lock( getStateMutex() );
        button = state.buttons;
    }

    /**
     * @brief Gets the current force acting on the tip.
     * @param force Vector that will store the force.
     */
    inline void getForce(std::vector<double> &force)
    {
        LockShared lock( getStateMutex() );
        force = state.force;
    }

    /**
     * @brief Sets the torque on the first three joints.
     * @param torque 3-elements vector with the torque values.
     */
    inline void setTorque(const std::vector<double> &torque)
    {
        LockUpgrade lock( getStateMutex() );
        if (!state.control_on)
        {
            return;
        }
        LockUpgradeToUnique lock_unique(lock);
        state.control = torque;
    }

    /**
     * @brief Resets the torque on the first three joints.
     * @see setTorque
     */
    inline void resetTorque()
    {
        std::vector<double> null_torque(3,0);
        OmniBase::setTorque(null_torque);
    }

    /**
     * @brief Chooses if the control on the first three joints is on.
     * @param enable True to enable. False otherwise.
     */
    inline void enableControl(bool enable)
    {
        LockUnique lock( getStateMutex() );
        state.control_on = enable;
    }

    /**
     * @brief Gets the current joint velocities.
     * @param vel Vector that will store the velocities.
     */
    inline void getJointVelocities(std::vector<double> &vel)
    {
        LockShared lock( getStateMutex() );
        vel = state.velocities;
    }

    /**
     * @brief Gets the current tip position with respect to the robot's base frame.
     * @param pos Vector that will store the position.
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

// ROS Callbacks
public:
    void torqueCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void enableControlCallback(const std_msgs::Bool::ConstPtr& msg);
};
