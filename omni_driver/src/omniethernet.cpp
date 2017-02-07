#include "../include/omniethernet.h"

int OmniEthernet::calibrationStyle = 0;

OmniEthernet::OmniEthernet(const std::string &name) :
    OmniBase(name)
{
}

void OmniEthernet::getJointAnglesFromDriver()
{
    // Using HD_CURRENT_ENCODER_VALUES instead of HD_CURRENT_JOINT_ANGLES
    // because the latter is not working properly (at all)
    HDlong encoders_values[6];
    hdGetLongv(HD_CURRENT_ENCODER_VALUES, encoders_values);

    double encoder2deg2rad = 0.024 * M_PI / 180;
    double gimbal2deg2rad = 0.0735 * M_PI / 180;
    state.angles[0] = encoder2deg2rad * encoders_values[0];
    state.angles[1] = encoder2deg2rad * encoders_values[1];
    state.angles[2] = encoder2deg2rad * (encoders_values[1] + encoders_values[2]);
    state.angles[3] = gimbal2deg2rad * (encoders_values[3] -2048);
    state.angles[4] = 0.0909*M_PI/180 * (encoders_values[4] -2563);
    state.angles[5] = gimbal2deg2rad * (encoders_values[5] -2259);
}

void OmniEthernet::autoCalibration()
{
    int supportedCalibrationStyles;
    HDErrorInfo error;

    hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
    if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET)
    {
        OmniEthernet::calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
        ROS_INFO("HD_CALIBRATION_ENCODER_RESET..");
    }
    if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL)
    {
        OmniEthernet::calibrationStyle = HD_CALIBRATION_INKWELL;
        ROS_INFO("HD_CALIBRATION_INKWELL..");
    }
    if (supportedCalibrationStyles & HD_CALIBRATION_AUTO)
    {
        OmniEthernet::calibrationStyle = HD_CALIBRATION_AUTO;
        ROS_INFO("HD_CALIBRATION_AUTO..");
    }

    if (OmniEthernet::calibrationStyle == HD_CALIBRATION_ENCODER_RESET)
    {
        do
        {
            hdUpdateCalibration(OmniEthernet::calibrationStyle);
            ROS_INFO("Calibrating.. (put stylus in well)");
            if (HD_DEVICE_ERROR(error = hdGetError()))
            {
                hduPrintError(stderr, &error, "Reset encoders reset failed.");
                break;
            }
        }
        while (hdCheckCalibration() != HD_CALIBRATION_OK);
        ROS_INFO("Calibration complete.");
        this->state.calibrated = true;
    }

    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
    {
        ROS_INFO("Please place the device into the inkwell for calibration.");
    }

    this->state.calibrated = true;
}

void OmniEthernet::forceCallback(const omni_driver::OmniFeedback::ConstPtr &omnifeed)
{
    ////////////////////Some people might not like this extra damping, but it
    ////////////////////helps to stabilize the overall force feedback. It isn't
    ////////////////////like we are getting direct impedance matching from the
    ////////////////////omni anyway
    this->state.force[0] = omnifeed->force.x - 0.001 * this->state.velocities[0];
    this->state.force[1] = omnifeed->force.y - 0.001 * this->state.velocities[1];
    this->state.force[2] = omnifeed->force.z - 0.001 * this->state.velocities[2];

    this->state.lock_pos[0] = omnifeed->position.x;
    this->state.lock_pos[1] = omnifeed->position.y;
    this->state.lock_pos[2] = omnifeed->position.z;
}

bool OmniEthernet::connect()
{
    // The name must be the same configured in the geomagic software.
    hHD = hdInitDevice("Phantom Omni");
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        //hduPrintError(stderr, &error, "Failed to initialize haptic device");
        ROS_ERROR("Failed to initialize haptic device"); //: %s", &error);
        return -1;
    }

    ROS_INFO("Found %s.", hdGetString(HD_DEVICE_MODEL_TYPE));
    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        ROS_ERROR("Failed to start the scheduler"); //, &error);
        return -1;
    }
    autoCalibration();
    hdScheduleAsynchronous(&OmniEthernet::callback, this, HD_MAX_SCHEDULER_PRIORITY);
    state.connected = true;
}

bool OmniEthernet::connected()
{
    return this->state.connected;
}

void OmniEthernet::disconnect()
{
    ROS_INFO("Ending Session....");
    hdStopScheduler();
    hdDisableDevice(hHD);
}

HDCallbackCode HDCALLBACK OmniEthernet::callback(void *pdata)
{
    OmniEthernet *omni = static_cast<OmniEthernet*>(pdata);
    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE)
    {
      ROS_DEBUG("Updating calibration...");
      hdUpdateCalibration(OmniEthernet::calibrationStyle);
    }

    hdBeginFrame(hdGetCurrentDevice());
    //Get angles, set forces
    double position[3];
    hdGetDoublev(HD_CURRENT_POSITION, position);
    std::copy(position, position + 3, omni->state.position.begin());
    //
    omni->getJointAnglesFromDriver();
    hduVector3Dd vel_buff(0, 0, 0);

    //todo velocity
    /*vel_buff = (omni->state.position * 3. - 4. * omni->state.pos_hist1
            + omni->state.pos_hist2) / 0.002;  //mm/s, 2nd order backward dif
    omni->state.velocities = (.2196 * (vel_buff + omni->state.vel_inp3)
            + .6588 * (omni->state.vel_inp1 + omni->state.vel_inp2)) / 1000.0
            - (-2.7488 * omni->state.vel_out1 + 2.5282 * omni->state.vel_out2
                    - 0.7776 * omni->state.vel_out3);  //cutoff freq of 20 Hz
    omni->state.pos_hist2 = omni->state.pos_hist1;
    omni->state.pos_hist1 = omni->state.position;
    omni->state.vel_inp3 = omni->state.vel_inp2;
    omni->state.vel_inp2 = omni->state.vel_inp1;
    omni->state.vel_inp1 = vel_buff;
    omni->state.vel_out3 = omni->state.vel_out2;
    omni->state.vel_out2 = omni->state.vel_out1;
    omni->state.vel_out1 = omni->state.velocities;*/
    if (omni->state.lock == true)
    {
        for (int k = 0; k<3; ++k)
        {
            omni->state.force[k] = 0.04 * (omni->state.lock_pos[k] - omni->state.position[k])
                    - 0.001 * omni->state.velocities[k];
        }
    }
    const double force[3] = {omni->state.force[0],
                           omni->state.force[1],
                           omni->state.force[2]};
    hdSetDoublev(HD_CURRENT_FORCE, force);
    //Get buttons
    int nButtons = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    omni->state.buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
    omni->state.buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

    hdEndFrame(hdGetCurrentDevice());

    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Error during main scheduler callback");
        if (hduIsSchedulerError(&error))
            return HD_CALLBACK_DONE;
    }

    return HD_CALLBACK_CONTINUE;
}
