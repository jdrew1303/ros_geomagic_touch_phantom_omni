#include "../include/omniethernet.h"


OmniEthernet::OmniEthernet(const std::string &name) :
    OmniBase(name)
{
}

void OmniEthernet::getJointAnglesFromDriver(OmniState *state)
{
    // Using HD_CURRENT_ENCODER_VALUES instead of HD_CURRENT_JOINT_ANGLES
    // because the latter is not working properly (at all)
    HDlong encoders_values[6];
    hdGetLongv(HD_CURRENT_ENCODER_VALUES, encoders_values);

    double encoder2deg2rad = 0.024 * M_PI / 180;
    double gimbal2deg2rad = 0.0735 * M_PI/180;
    state->angles[0] = encoder2deg2rad * encoders_values[0];
    state->angles[1] = encoder2deg2rad * encoders_values[1];
    state->angles[2] = encoder2deg2rad * (encoders_values[1] + encoders_values[2]);
    state->angles[3] = gimbal2deg2rad * (encoders_values[3] -2048);
    state->angles[4] = 0.0909*M_PI/180 * (encoders_values[4] -2563);
    state->angles[5] = gimbal2deg2rad * (encoders_values[5] -2259);
}

void OmniEthernet::autoCalibration()
{
    int supportedCalibrationStyles;
    HDErrorInfo error;

    hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
    if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET)
    {
        calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
        ROS_INFO("HD_CALIBRATION_ENCODER_RESE..");
    }
    if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL)
    {
        calibrationStyle = HD_CALIBRATION_INKWELL;
        ROS_INFO("HD_CALIBRATION_INKWELL..");
    }
    if (supportedCalibrationStyles & HD_CALIBRATION_AUTO)
    {
        calibrationStyle = HD_CALIBRATION_AUTO;
        ROS_INFO("HD_CALIBRATION_AUTO..");
    }
    if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET)
    {
        do
        {
            hdUpdateCalibration(calibrationStyle);
            ROS_INFO("Calibrating.. (put stylus in well)");
            if (HD_DEVICE_ERROR(error = hdGetError()))
            {
                hduPrintError(stderr, &error, "Reset encoders reset failed.");
                break;
            }
        }
        while (hdCheckCalibration() != HD_CALIBRATION_OK);
        ROS_INFO("Calibration complete.");
    }
    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
    {
        ROS_INFO("Please place the device into the inkwell for calibration.");
    }
}

void OmniEthernet::forceCallback(const omni_driver::OmniFeedbackConstPtr &omnifeed)
{
    ////////////////////Some people might not like this extra damping, but it
    ////////////////////helps to stabilize the overall force feedback. It isn't
    ////////////////////like we are getting direct impedance matching from the
    ////////////////////omni anyway
    state->force[0] = omnifeed->force.x - 0.001 * state->velocities[0];
    state->force[1] = omnifeed->force.y - 0.001 * state->velocities[1];
    state->force[2] = omnifeed->force.z - 0.001 * state->velocities[2];

    state->lock_pos[0] = omnifeed->position.x;
    state->lock_pos[1] = omnifeed->position.y;
    state->lock_pos[2] = omnifeed->position.z;
}

bool OmniEthernet::connect()
{
    HDErrorInfo error;
    HHD hHD;
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
    HHD_Auto_Calibration();
}

bool OmniEthernet::connected()
{
    //todo
}

void OmniEthernet::disconnect()
{
    ROS_INFO("Ending Session....");
    hdStopScheduler();
    hdDisableDevice(hHD);
}

HDCallbackCode HDCALLBACK omni_state_callfback(void *pUserData)
{
    OmniState *omni_state = static_cast<OmniState *>(pUserData);
    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE)
    {
      ROS_DEBUG("Updating calibration...");
      hdUpdateCalibration(calibrationStyle);
    }

    hdBeginFrame(hdGetCurrentDevice());
    //Get angles, set forces
    hdGetDoublev(HD_CURRENT_POSITION, omni_state->position);
    getJointAnglesFromDriver(omni_state);
    hduVector3Dd vel_buff(0, 0, 0);
    vel_buff = (omni_state->position * 3 - 4 * omni_state->pos_hist1
            + omni_state->pos_hist2) / 0.002;  //mm/s, 2nd order backward dif
    omni_state->velocities = (.2196 * (vel_buff + omni_state->vel_inp3)
            + .6588 * (omni_state->vel_inp1 + omni_state->vel_inp2)) / 1000.0
            - (-2.7488 * omni_state->vel_out1 + 2.5282 * omni_state->vel_out2
                    - 0.7776 * omni_state->vel_out3);  //cutoff freq of 20 Hz
    omni_state->pos_hist2 = omni_state->pos_hist1;
    omni_state->pos_hist1 = omni_state->position;
    omni_state->vel_inp3 = omni_state->vel_inp2;
    omni_state->vel_inp2 = omni_state->vel_inp1;
    omni_state->vel_inp1 = vel_buff;
    omni_state->vel_out3 = omni_state->vel_out2;
    omni_state->vel_out2 = omni_state->vel_out1;
    omni_state->vel_out1 = omni_state->velocities;
    if (omni_state->lock == true)
    {
        omni_state->force = 0.04 * (omni_state->lock_pos - omni_state->position)
                - 0.001 * omni_state->velocities;
    }

    hdSetDoublev(HD_CURRENT_FORCE, omni_state->force);
    //Get buttons
    int nButtons = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    omni_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
    omni_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

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

