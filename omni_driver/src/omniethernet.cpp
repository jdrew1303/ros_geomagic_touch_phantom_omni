#include "../include/omniethernet.h"

int OmniEthernet::calibrationStyle = 0;

OmniEthernet::OmniEthernet(const std::string &name, const std::string &path_urdf, const std::string &path_srdf) :
    OmniBase(name, path_urdf, path_srdf)
{
    this->resetTorque();
}

OmniEthernet::~OmniEthernet()
{
    this->disconnect();
}

void OmniEthernet::getJointAnglesFromDriver()
{
    // Using HD_CURRENT_ENCODER_VALUES instead of HD_CURRENT_JOINT_ANGLES
    // because the latter is not working properly (at all)
    HDlong encoders_values[6];
    hdGetLongv(HD_CURRENT_ENCODER_VALUES, encoders_values);

    double encoder2deg2rad = 0.024 * M_PI / 180;
    double gimbal2deg2rad = 0.0735 * M_PI / 180;

    // Getting time difference between two consecutive reading.
//    state.time_last_angle_acquisition = state.time_current_angle_acquisition;
    Clock clock;
    state.time_current_angle_acquisition = clock.local_time();

    // All angles should be ~= 0 when docked.
    state.angles[0] =   encoder2deg2rad *  encoders_values[0];
    state.angles[1] = -(encoder2deg2rad *  encoders_values[1] + 0.2760412744 );
    state.angles[2] =  (encoder2deg2rad * (encoders_values[1] + encoders_values[2]) + 0.6572211831 );
    state.angles[3] = -(gimbal2deg2rad  * (encoders_values[3] - 2048));
    state.angles[4] = -(gimbal2deg2rad  * (encoders_values[4] - 600) * 1.2 - 3.160347962);
    state.angles[5] =  (gimbal2deg2rad  * (encoders_values[5] -2259) - 0.04233296100712246 );

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
}

bool OmniEthernet::connect()
{

/*######################################################################################
             The name must be the same configured in the geomagic software!
  ######################################################################################*/
    hHD = hdInitDevice("Phantom Omni");
/*######################################################################################*/

    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        //hduPrintError(stderr, &error, "Failed to initialize haptic device");
        ROS_ERROR("Failed to initialize haptic device"); //: %s", &error);
        return 0;
    }

    ROS_INFO("Found %s.", hdGetString(HD_DEVICE_MODEL_TYPE));
    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        ROS_ERROR("Failed to start the scheduler"); //, &error);
        return 0;
    }
    autoCalibration();
    handle_callback = hdScheduleAsynchronous(&OmniEthernet::callback, this, HD_MAX_SCHEDULER_PRIORITY);
    state.connected = true;
    return 1;
}

bool OmniEthernet::connected()
{
    return this->state.connected;
}

void OmniEthernet::wakeup()
{
}

void OmniEthernet::disconnect()
{
    ROS_INFO("Ending Session....");
    hdStopScheduler();
    hdUnschedule(handle_callback);
    hdDisableDevice(hHD);
}

HDCallbackCode OmniEthernet::callback(void *pdata)
{
    OmniEthernet *omni = static_cast<OmniEthernet*>(pdata);

    OmniBase::LockUnique lock( omni->getStateMutex() );

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
    omni->getJointAnglesFromDriver();

    // Calculates forward kinematics and velocities.
    omni->updateRobotState();

    if (omni->state.lock == true)
    {
        for (int k = 0; k<3; ++k)
        {
            omni->state.control[k] = 0.04 * (omni->state.lock_pos[k] - omni->state.position[k])
                    - 0.001 * omni->state.velocities[k];
        }
    }
    hdSetLongv(HD_CURRENT_MOTOR_DAC_VALUES, omni->force_output);
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

void OmniEthernet::mapTorque()
{
    force_output[0] = -state.control[0] * 32767;
    force_output[1] = state.control[1] * 32767;
    force_output[2] = state.control[2] * 32767;
}

