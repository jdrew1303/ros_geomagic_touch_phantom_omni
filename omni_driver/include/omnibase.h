#pragma once


#include <vector>

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include "omni_driver/OmniButtonEvent.h"

class OmniBase
{

private:

    boost::shared_mutex mutex_state;                        ///< Mutex state returned by @link getMutexState().

protected:
    typedef boost::posix_time::microsec_clock Clock;
    typedef boost::posix_time::ptime Time;
    typedef boost::posix_time::time_duration TimeDuration;

    struct OmniState                                        ///< The state structure with relevant data.
    {
        std::vector<double> angles;
        std::vector<double> angles_docked;
        std::vector<double> angles_last;
        std::vector<double> control;
        std::vector<double> force;
        std::vector<double> lock_pos;
        std::vector<double> orientation;
        std::vector<double> position;
        std::vector<double> pos_hist1;
        std::vector<double> pos_hist2;
        std::vector<double> velocities;
        std::vector<double> vel_inp1;
        std::vector<double> vel_inp2;
        std::vector<double> vel_inp3;
        std::vector<double> vel_out1;
        std::vector<double> vel_out2;
        std::vector<double> vel_out3;
        std::vector<bool>   buttons;
        bool control_on = false;
        bool connected = false;
        bool calibrated = false;
        bool lock = false;
        unsigned int seq = 0;
        Time stamp;

        OmniState()
        {
            // Joints
            angles.resize(6);
            angles_docked.resize(6);
            angles_last.resize(6);
            velocities.resize(6);

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


    ros::Publisher pub_joint;                   ///< Joint ROS publisher.
    sensor_msgs::JointState joint_state;

    ros::Publisher pub_pose;                    ///< Pose ROS publisher.
    geometry_msgs::PoseStamped pose_stamped;

    ros::Publisher pub_button;                  ///< Button ROS publisher.
    omni_driver::OmniButtonEvent button_event;


    bool last_buttons[2];                       ///< Needed for "Button Clicked" logic.
    bool enable_force_flag;                     ///< Needed for resetting the internal enable control.

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

public:
    explicit OmniBase(const std::string &name = "");

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
        LockUnique lock( getStateMutex() );
        state.control[0] = (torque[0] + 1) / 2 * 4095;
        state.control[1] = (torque[1] + 1) / 2 * 4095;
        state.control[2] = (torque[2] + 1) / 2 * 4095;
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
    inline void enableControl(bool enable)
    {
        {
            // Creating a new scope to avoid a deadlock
            LockUnique lock( getStateMutex() );
            state.control_on = enable;
            enable_force_flag = enable;
        }
        this->resetTorque();
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
};

typedef boost::shared_ptr<OmniBase> OmniBasePtr;
