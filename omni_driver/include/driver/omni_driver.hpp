#pragma once

#include "../include/math/omni_math.hpp"
#include "link.hpp"

class OmniDriver {
    private:

    const bool enable_moving_average;

    OmniMath omni_math(enable_moving_average);

    struct Links {
        Link base = Link("base");
        Link torso = Link("torso");
        Link upper_arm = Link("upper_arm");
        Link lower_arm = Link("lower_arm");
        Link wrist = Link("wrist");
        Link tip = Link("tip");
        Link stylus = Link("stylus");
        Link end_effector = Link("end_effector");
    };

    Links links;

    Eigen::Matrix3d rot_link_to_teleop;

    double force_feedback_gain;

    double joint_states_gain;

    double twist_gain;

    double last_published_joint5_velocity;

    bool teleop_master;

    static const unsigned int VELOCITIES_FILTER_SIZE = 5;

    static const unsigned int MAX_FREEZE_COUNT = 100;

    boost::shared_mutex mutex_state;                        ///< Mutex state returned by @link getMutexState().

    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;
    robot_state::JointModelGroup* joint_model_group;
    const robot_state::LinkModel* end_effector_link_model;

    bool last_buttons[2];                       ///< Needed for "Button Clicked" logic.
    bool enable_force_flag;                     ///< Needed for resetting the internal enable control.
    double velocity_filter_minimum_dt;          ///< TODO value in milliseconds

    /**
     * @brief Gets the mutex used for accessing the robot's state.
     * @return The mutex.
     */
    inline boost::shared_mutex & getStateMutex() {
        return mutex_state;
    }

    /**
     * @brief Called by the base after a new control value has been received.
     * @see setTorque
     */
    virtual std::vector<double> mapTorque(std::vector<double>) = 0;

    protected:

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

    /**
     * Must only be called internally after updating joint angles to simulate acquiring calculated data.
     */
    void updateStateWithCalculations(
        std::vector<double> joint_angles) {

        Eigen::VectorXd joint_angles(6);
        joint_angles << state.angles[0],
                        state.angles[1],
                        state.angles[2],
                        state.angles[3],
                        state.angles[4],
                        state.angles[5];
        kinematic_state->setJointGroupPositions(joint_model_group, joint_angles);
        

    }

    void updateState()
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
    calculateVelocities();
    std::vector<double> filtered_velocities;
    filterVelocities(filtered_velocities);
    getEffectorVelocity();
}

    public:
    /**
     * @brief OmniBase constructor, sets some members and prepares ros topics and publishers.
     * @param name Reference to string of omni name.
     */
    explicit OmniBase(const std::string &name, const std::string &path_urdf, const std::string &path_srdf);

    /**
     * @brief OmniBase constructor, sets some members and prepares ros topics and publishers.
     * @param name Reference to string of omni name.
     * @param velocity_filter_minimum_dt Minimum amount of time that should have passed to compute the robot velocity.
     */
    OmniBase(const std::string &name,  const std::string &path_urdf, const std::string &path_srdf, double velocity_filter_minimum_dt);

    /**
     * @brief Gets the robot's state.
     * @return The robot's state.
     */
    inline OmniState getState() {
        LockShared lock( getStateMutex() );
        return state;
    }

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
     * @brief Chooses if the control on the first three joints is on.
     * @param enable True to enable. False otherwise.
     */
    virtual void enableControl(bool enable) = 0;

    /**
     * @brief Gets the current joint angles.
     * @param angles std::Vector that will store the angles.
     */
    inline std::vector<double> getJointAngles() {
        // Protect the critical section.
        return getState().angles;
    }

    /**
     * @brief Gets the current joint angles.
     * @param angles Eigen::VectorXd that will store the angles.
     */
    inline Eigen::VectorXd getJointAngles(Eigen::VectorXd angles) {
        // Protect the critical section.
        OmniState s = getState();
        angles << s.angles[0],
            s.angles[1],
            s.angles[2],
            s.angles[3],
            s.angles[4],
            s.angles[5];
    }

    /**
     * @brief Gets the current buttons' state.
     * @param button std::Vector that will store the states.
     */
    inline std::vector<bool> getButtonsState() {
        return getState().buttons;
    }

    /**
     * @brief Gets the current force acting on the tip.
     * @param force std::vector that will store the force.
     */
    inline std::vector<double> getForce() {
        return getState().force;
    }

    /**
     * @brief Sets the torque on the first three joints.
     * @param torque 3-elements std::vector with the torque values.
     */
    inline void setTorque(std::vector<double> torque) {
        LockUnique lock( getStateMutex() );
        auto t = mapTorque(omni_math.saturate(torque, -1.0, 1.0));
        state.control = t;
    }

    /**
     * @brief Resets the torque on the first three joints.
     * @see setTorque
     */
    inline void resetTorque() {
        std::vector<double> null_torque(3,0);
        this->setTorque(null_torque);
    }

    /**
     * @brief Gets the current joint velocities.
     * @param vel std::vector that will store the velocities.
     */
    inline std::vector<double> getJointVelocities() {
        return getState().velocities;
    }

    /**
     * @brief Gets the current joint velocities.
     * @param vel Eigen::VectorXd that will store the velocities.
     */
    inline Eigen::VectorXd getJointVelocities() {
        auto s = getState();
        Eigen::VectorXd vel;
        vel << s.velocities[0],
               s.velocities[1],
               s.velocities[2],
               s.velocities[3],
               s.velocities[4],
               s.velocities[5];
        return vel;
    }

    /**
     * @brief Gets the current tip position with respect to the robot's base frame.
     * @param pos std::vector that will store the position.
     */
    inline std::vector<double> getTipPosition() {
        return getState().position;
    }

    /**
     * @brief Gets the current tip orientation with respect to the robot's base frame.
     * @param ori Vector that will store the orientation as a quaternion.
     */
    inline std::vector<double> getTipOrientation() {
        return getState().orientation;
    }

    /**
     * @brief Gets the current tip position and orientation with respect to the robot's base frame.
     * @param pos Vector that will store the position.
     * @param ori Vector that will store the orientation as a quaternion.
     * @see getTipPosition, getTipOrientation
     */
    inline Eigen::Affine3d getTipAffinity() {
        omni_math.forwarKinematics(kinematic_state, links.end_effector.name);
    }

    /**
     * @brief Checks if the device has been calibrated.
     * @return True if calibrated. False otherwise.
     */
    inline bool calibrated() {
        return getState().calibrated;
    }

};