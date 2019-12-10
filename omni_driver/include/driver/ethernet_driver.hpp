#pragma once

#include "../include/driver/omni_ethernet.hpp"

#include "omnibase.h"
#include "ros/ros.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <arpa/inet.h>
#include <libraw1394/csr.h>
#include <vector>
#include <string>
#include <stdint.h>
#include <libraw1394/raw1394.h>
#include <boost/thread.hpp>

#define linux
#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#undef linux

#include "omni_driver/OmniFeedback.h"



class EthernetDriver : public OmniBase {
    private:
    HDErrorInfo error;
    HHD hHD;
    HDSchedulerHandle handle_callback;
    HDlong force_output[3];

    /**
     * @brief Calibrates the device using one of the supported styles.
     */
    void autoCalibration();

    /**
     * @brief ROS subsricber callback to set the device's haptic feedback.
     * @param omnifeed Receives a ROS message.
     */
    void forceCallback(const omni_driver::OmniFeedback::ConstPtr& omnifeed);

    /**
     * @brief Gets the angles from the encoder and sets them to OmniState.
     */
    void getJointAnglesFromDriver();

    /**
     * @brief Callback that will be called at every iteration, getting data from the device and locking the device position if necessary.
     * @param pdata Receives a pointer to an OmniEthernet object.
     * @return Returns 1 if the communication should continue, 0 otherwise.
     */
    static HDCallbackCode callback(void *pdata);

protected:
    void mapTorque();

public:
    static int calibrationStyle;

    /**
     * @brief Connects and calibrates to the device, starting the communication.
     * @return Return 1 if the connection was succesfull, 0 otherwise.
     */
    bool connect();

    /**
     * @brief Used to check if a device is connected.
     * @return Returns 1 if a device connected, 0 otherwise.
     */
    bool connected();

    /**
     * @brief Disconnects the device, called by destructor.
     */
    void disconnect();

    /**
     * @brief OmniEthernet constructor, calls OmniBase constructor with same parameter.
     * @param name Reference to string of omni name.
     */
    OmniEthernet(
        const std::string &path_urdf,
        const std::string &path_srdf,
        double force_feedback_gain,
        double joint_states_gain,
        double twist_gain,
        bool teleop_master,
        double velocity_filter_minimum_dt);
    ~OmniEthernet();

    inline void enableControl(bool enable) {
        {
            // Creating a new scope to avoid a deadlock
            LockUnique lock( getStateMutex() );
            state.control_on = enable;
            enable_force_flag = enable;
        }
        if (enable) {
            hdDisable(HD_FORCE_OUTPUT);
            hdEnable(HD_FORCE_OUTPUT);
        }
        else {
            hdDisable(HD_FORCE_OUTPUT);
        }

        this->resetTorque();
    }

};