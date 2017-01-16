#pragma once


#include <vector>

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>


class OmniBase
{
protected:
    typedef boost::posix_time::microsec_clock Clock;
    typedef boost::posix_time::ptime Time;
    typedef boost::posix_time::time_duration TimeDuration;

    struct OmniState
    {
        std::vector<double> angles;
        std::vector<double> angles_last;
        std::vector<double> velocities;
        std::vector<double> force;
        std::vector<double> control;
        std::vector<double> position;
        std::vector<double> orientation;

        bool control_on = false;

        unsigned int seq = 0;
        Time stamp;
    };

private:
    boost::shared_mutex mutex_state;
    OmniState state;

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
     * @brief This method should be set as the driver's callback. It performs housekeeping
     * tasks and calls the implementation specific {@link callback} method after casting \p data
     * to OmniState *.
     *
     * The \p data parameter type is a pointer to void because this is the usual type
     * required by most drivers. However, this should castable to a pointer to OmniState.
     *
     * @param data Data state stored by the driver.
     */
    void driverCallback(void *data);

    /**
     * @brief Implementation specific callback function called from {@link driverCallback}.
     *
     * All driver specific calls to set and get information from the robot should be coded
     * in this method.
     *
     * @param state The current robot's state.
     */
    virtual void callback(OmniState *state) = 0;

    /**
     * @brief Gets the robot's state.
     * @return The robot's state.
     */
    inline OmniState * getState()
    {
        return &state;
    }

public:
    OmniBase();

    /**
     * @brief Attempts to connect to the robot.
     * @return True if the connection succeeds. False otherwise.
     */
    virtual bool connect() = 0;

    /**
     * @brief Attempts to close the connection to the robot.
     * @return True if the connection was closed. False otherwise.
     */
    virtual bool disconnect() = 0;

    /**
     * @brief Gets the current joint angles.
     * @param angles Vector that will store the angles.
     */
    inline void getJointAngles(std::vector<double> &angles)
    {
        LockShared lock( getStateMutex() );
        angles = state.angles;
    }

    /**
     * @brief Gets the current force acting on the tip.
     * @param force Vector that will store the force.
     */
    virtual void getForce(std::vector<double> &force)
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
};
