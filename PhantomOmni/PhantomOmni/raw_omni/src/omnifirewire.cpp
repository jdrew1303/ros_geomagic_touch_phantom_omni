#include "../include/omnifirewire.h"


void OmniFirewire::callback(OmniBase::OmniState *state)
{
}


bool OmniFirewire::connect()
{
        // Make sure the device is closed at this point.
        disconnect();

        // Find the Phantom Omni.
        const std::vector<OmniInfo>& omnis = enumerate_omnis();
        typedef std::vector<OmniInfo>::const_iterator CIT;
        for (CIT it = omnis.begin(); it != omnis.end(); ++it) {
            if (it->serial == serial_number_) {
                // Serial number matched: found the omni!
                port_ = it->port;
                node_ = it->node;

                break;
            }
        }
        if (node_ == -1) {
            // Failed to find the omni.
            return false;
        }

        // Create a 1394 handle.
        handle_ = raw1394_new_handle();
        raw1394_set_userdata(handle_, this);

        // Set the port.
        if (raw1394_set_port(handle_, port_) == -1) {
            return false;
        }

        // Initialize the Tx buffer.
        tx_iso_buffer_.force_x     = 0x07ff;
        tx_iso_buffer_.force_y     = 0x07ff;
        tx_iso_buffer_.force_z     = 0x07ff;
        tx_iso_buffer_.status.bits = 0x53c0;
        tx_iso_buffer_.padding1    = 0x0000;
        tx_iso_buffer_.padding2    = 0x0000;
        tx_iso_buffer_.status.dock_led0 = 0;
        tx_iso_buffer_.status.dock_led1 = 1;
        tx_iso_buffer_.status.force_enabled = 1;

        // Reset the pot filter counter.
        pot_filter_accum_[0] = 0.0;
        pot_filter_accum_[1] = 0.0;
        pot_filter_accum_[2] = 0.0;
        pot_filter_count_ = RAW_OMNI_POT_FILTER_TAPS;

        // Start isochronous transmission.
        if (!start_isochronous_transmission()) {
            return false;
        }
        return true;
}


void OmniFirewire::disconnect()
{
        // Turn the dock LED off.
        tx_iso_buffer_.status.dock_led0 = 0;
        tx_iso_buffer_.status.dock_led1 = 0;
        calibrated_ = false;

        // Stop isochronous data transmission.
        stop_isochronous_transmission();

        port_ = -1;
        node_ = -1;

        if (handle_ != NULL)
        {
            raw1394_destroy_handle(handle_);
            handle_ = NULL;
        }

}

void get_current_joint_angles(std::vector<double>& angles)
{
    // Protect the critical section.
    pthread_mutex_lock(&iso_mutex_);

    angles.clear();
    angles.insert(angles.end(), current_joint_angles_, current_joint_angles_
            + sizeof(current_joint_angles_) / sizeof(*current_joint_angles_));

    pthread_mutex_unlock(&iso_mutex_);
}

~OmniFirewire::OmniFirewire(const std::string& serial_number)
{
    driver_.close();
}

void timerHandler(const ros::TimerEvent& event)
{
    if (driver_.opened() && !driver_.calibrated()) {
        // Phantom Omni is not open or calibrated. Don't publish.
        return;
    }

    // Get the joint angles from the omni.
    std::vector<double> joint_angles;
    driver_.get_current_joint_angles(joint_angles);

    // Publish the joint state.
    joint_state_.header.stamp = ros::Time::now();
    for (int i = 0; i < 6; i++) {
        joint_state_.position[i] = joint_angles[i];
    }
    //the next line is a necessary to recalibrate potentiometer #2
    joint_state_.position[4]=(joint_state_.position[4]+0.27+(joint_state_.position[4]+1.73)/1.72);

    joint_pub_.publish(joint_state_);

    // Publish the end effector pose.
    pose_stamped_.header.stamp = ros::Time::now();
    pose_stamped_.pose.position.x = 0.0;
    pose_stamped_.pose.orientation.w = 1.0;
    pose_pub_.publish(pose_stamped_);

    // Publish the button state.
    std::vector<bool> button_state;
    driver_.get_current_button_state(button_state);
    button_event_.grey_button = button_state[0];
    button_event_.white_button = button_state[1];
    //
    button_event_.grey_button_clicked = !last_buttons[0] && button_state[0];
    button_event_.white_button_clicked = !last_buttons[1] && button_state[1];
    last_buttons[0] = button_state[0];
    last_buttons[1] = button_state[1];
    //
    button_pub_.publish(button_event_);
}

bool run()
{
    // Open Phantom Omni.
    if (!driver_.open()) {
        ROS_ERROR("Failed to open Phantom Omni.");
        return false;
    }
    ROS_INFO("%s (%s): opened.",
            name_.c_str(), driver_.serial_number().c_str());

    // Spin.
    ros::spin();

    return true;
}

void firewireMain(int argc, char** argv)
{
    // Initialize ROS.
    ros::init(argc, argv, "raw_omni");

    // Get the omni parameters.
    std::string omni_name;
    ros::param::param<std::string>("~omni_name", omni_name, "");
    std::string omni_serial;
    ros::param::param<std::string>("~omni_serial", omni_serial, "11129400000");

    // List the serial number of each Phantom Omni connected.
    const std::vector<RawOmniDriver::OmniInfo>& omnis = RawOmniDriver::enumerate_omnis();
    std::ostringstream serial_numbers;
    for (size_t i = 0; i < omnis.size(); i++) {
        serial_numbers << omnis[i].serial << " ";
    }
    ROS_INFO("Phantom Omni Serial Numbers: %s", serial_numbers.str().c_str());

    // Start the node.
    RawOmniNode node(omni_name, omni_serial);
    node.run();
}
