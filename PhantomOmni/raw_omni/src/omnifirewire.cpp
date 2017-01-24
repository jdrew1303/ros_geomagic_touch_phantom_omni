#include "../include/omnifirewire.h"

OmniFirewire::OmniFirewire(const std::string &serial_number)
        : state_(RAW_OMNI_STATE_READY),
        serial_number_(serial_number),
        handle_(NULL), tx_handle_(NULL), rx_handle_(NULL),
        port_(-1), node_(-1), tx_iso_channel_(-1), rx_iso_channel_(-1),
        calibrated_(false), enable_force_flag(false)
{
    // Create mutex for data synchronization.
    pthread_mutex_init(&iso_mutex_, NULL);

    // Initialize the variables.
    std::memset(current_joint_angles_, 0, sizeof(current_joint_angles_));
    std::memset(home_joint_angles_, 0, sizeof(home_joint_angles_));
    std::memset(current_buttons_, 0, sizeof(current_buttons_));
    //
    this->reset_force();
}

~OmniFirewire(const std::string& serial_number)
{
    driver_.disconnect();
}

void OmniFirewire::callback(OmniBase::OmniState *state)
{

}

enum raw1394_iso_disposition callbackWrite(raw1394handle_t handle,unsigned char* data, unsigned int* len, unsigned char* tag, unsigned char* sy, int cycle, unsigned int dropped)
{
*len = 0;
*tag = 0;
*sy = 0;
// Get the RawOmniDriver object for this handle.
RawOmniDriver* omni = reinterpret_cast<RawOmniDriver*>(raw1394_get_userdata(handle));
if (omni == NULL)
{
    return RAW1394_ISO_OK;
}
// Edit force output
omni->tx_iso_buffer_.force_x = omni->force_output_[0];
omni->tx_iso_buffer_.force_y = omni->force_output_[1];
omni->tx_iso_buffer_.force_z = omni->force_output_[2];
    if (omni->enable_force_flag)
    {
        if (omni->tx_iso_buffer_.status.force_enabled == 1)
            omni->tx_iso_buffer_.status.force_enabled = 0;
            else
            {
            omni->tx_iso_buffer_.status.force_enabled = 1;
            omni->enable_force_flag = false;
            }
    }

// Copy data.
std::memcpy(data, &omni->tx_iso_buffer_, sizeof(omni->tx_iso_buffer_));
*len = sizeof(omni->tx_iso_buffer_);
*tag = 0;
*sy = 0;
return RAW1394_ISO_OK;
}

enum raw1394_iso_disposition callbackRead(raw1394handle_t handle, unsigned char* data, unsigned int len, unsigned char channel, unsigned char tag, unsigned char sy, unsigned int cycle, unsigned int dropped)
{
    // Get the OmniFirewire object for this handle.
    OmniFirewire* omni = reinterpret_cast<OmniFirewire*>(raw1394_get_userdata(handle));
    if (omni == NULL)
    {
        return RAW1394_ISO_OK;
    }

    const OmniFirewire::RxIsoBuffer* newbuf = reinterpret_cast<OmniFirewire::RxIsoBuffer*>(data);
    // Calibrate if docked.
    if (!newbuf->status.undocked)
    {
    // Calibrate.
    omni->home_joint_angles_[0] = 2.0 * M_PI *  newbuf->encoder_x / 15000.0;
    omni->home_joint_angles_[1] = 2.0 * M_PI * -newbuf->encoder_y / 15000.0 - 0.23;
    omni->home_joint_angles_[2] = 2.0 * M_PI *  newbuf->encoder_z / 15000.0 + 0.37;
    // Calibrated: the dock LED should be always on.
    omni->tx_iso_buffer_.status.dock_led0 = 1;
    omni->tx_iso_buffer_.status.dock_led1 = 1;
    omni->calibrated_ = true;
    }
    // Compute the encoder values.
    omni->current_joint_angles_[0] = 2.0 * M_PI
    *  newbuf->encoder_x / 15000.0 - omni->home_joint_angles_[0];
    omni->current_joint_angles_[1] = 2.0 * M_PI
    * -newbuf->encoder_y / 15000.0 - omni->home_joint_angles_[1];
    omni->current_joint_angles_[2] = 2.0 * M_PI
    *  newbuf->encoder_z / 15000.0 - omni->home_joint_angles_[2]
    - omni->current_joint_angles_[1];
    // Add the current gimbal pot value for filtering.
    omni->pot_filter_accum_[0] += (double)newbuf->gimbal_a_x
    / (newbuf->gimbal_a_x + newbuf->gimbal_b_x);
    omni->pot_filter_accum_[1] += (double)newbuf->gimbal_a_y
    / (newbuf->gimbal_a_y + newbuf->gimbal_b_y);
    omni->pot_filter_accum_[2] += (double)newbuf->gimbal_a_z
    / (newbuf->gimbal_a_z + newbuf->gimbal_b_z);

    if (--omni->pot_filter_count_ <= 0)
    {
        omni->pot_filter_count_ = RAW_OMNI_POT_FILTER_TAPS;

        // Normalize.
        omni->pot_filter_accum_[0] /= RAW_OMNI_POT_FILTER_TAPS;
        omni->pot_filter_accum_[1] /= RAW_OMNI_POT_FILTER_TAPS;
        omni->pot_filter_accum_[2] /= RAW_OMNI_POT_FILTER_TAPS;

        // Compute the gimbal values.
        omni->current_joint_angles_[3]
        =  5.48 * omni->pot_filter_accum_[0] - 2.74;
        omni->current_joint_angles_[4]
        = -5.28 * omni->pot_filter_accum_[1] + 2.64;
        omni->current_joint_angles_[5]
        =  4.76 * omni->pot_filter_accum_[2] - 2.38;

        // Reset the accumulator array.
        omni->pot_filter_accum_[0] = 0.0;
        omni->pot_filter_accum_[1] = 0.0;
        omni->pot_filter_accum_[2] = 0.0;
    }

    // Get the button state.
    omni->current_buttons_[0] = !newbuf->status.button1;
    omni->current_buttons_[1] = !newbuf->status.button2;

    return RAW1394_ISO_OK;
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

void OmniFirewire::get_current_joint_angles(std::vector<double>& angles)
{
    // Protect the critical section.
    pthread_mutex_lock(&iso_mutex_);

    angles.clear();
    angles.insert(angles.end(), current_joint_angles_, current_joint_angles_
            + sizeof(current_joint_angles_) / sizeof(*current_joint_angles_));

    pthread_mutex_unlock(&iso_mutex_);
}


void OmniFirewire::timerHandler(const ros::TimerEvent& event)
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

bool OmniFirewire::run()
{
    // Open Phantom Omni.
    if (!driver_.connect()) {
        ROS_ERROR("Failed to open Phantom Omni.");
        return false;
    }
    ROS_INFO("%s (%s): opened.",
            name_.c_str(), driver_.serial_number().c_str());

    // Spin.
    ros::spin();

    return true;
}

uint32_t OmniFirewire::pack_serial_number(const std::string& unpacked)
{
    // Serial number must have 11 characters.
    if (unpacked.size() != 11) {
        // Invalid length.
        return 0;
    }

    // Extract digits into 5 integers.
    uint32_t a, b, c, d, e;
    int filled = std::sscanf(unpacked.c_str(),
            "%1u%2u%2u%1u%5u", &a, &b, &c, &d, &e);
    if (filled != 5) {
        // Extraction failed.
        return 0;
    }

    uint32_t packed = 0x80000001;
    packed |= ((a & 0x007) << 23);  // a maps to Bit 25..23.
    packed |= ((b & 0x00f) << 19);  // b maps to Bit 22..19.
    packed |= ((c & 0x01f) << 14);  // c maps to Bit 18..14.
    packed |= ((d & 0x007) << 11);  // d maps to Bit 13..11.
    packed |= ((e & 0x3ff) <<  1);  // e maps to Bit 10..1.
    return packed;
}

std::string OmniFirewire::unpack_serial_number(uint32_t packed)
{
    // Unpack the variables based on the following rule:
    //     +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    //     |1|0|0|0|0|0|  a  |   b   |    c    |  d  |         e         |1|
    //     +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    //     |31           24|23           16|15            8|7             0|

    uint32_t a = (packed >> 23) & 0x007;
    uint32_t b = (packed >> 19) & 0x00f;
    uint32_t c = (packed >> 14) & 0x01f;
    uint32_t d = (packed >> 11) & 0x007;
    uint32_t e = (packed >>  1) & 0x3ff;

    std::stringstream unpacked;
    unpacked << std::setfill('0');
    unpacked << std::setw(1) << a;
    unpacked << std::setw(2) << b;
    unpacked << std::setw(2) << c;
    unpacked << std::setw(1) << d;
    unpacked << std::setw(5) << e;
    return unpacked.str();
}

bool OmniFirewire::start_isochronous_transmission()
{
    // Make sure raw1394 is ready.
    if (handle_ == NULL) {
        return false;
    }

    // Stop previous transmission.
    stop_isochronous_transmission();

    // Find a free isochronous channel number for Tx.
    for (int ch = 0; ch < 63; ch++) {
        if (raw1394_channel_modify(handle_, ch, RAW1394_MODIFY_ALLOC) == 0) {
            tx_iso_channel_ = ch;
            break;
        }
    }
    if (tx_iso_channel_ == -1) {
        // Channel not found. Abort.
        stop_isochronous_transmission();
        return false;
    }

    // Find a free isochronous channel number for Tx.
    for (int ch = 0; ch < 63; ch++) {
        if (raw1394_channel_modify(handle_, ch, RAW1394_MODIFY_ALLOC) == 0) {
            rx_iso_channel_ = ch;
            break;
        }
    }
    if (rx_iso_channel_ == -1) {
        // Channel not found. Abort.
        stop_isochronous_transmission();
        return false;
    }

    // Set the Tx isochronous channel.
    raw1394_write(handle_, node_, 0x1000, 1, (quadlet_t*)&tx_iso_channel_);

    // Set the Rx isochronous channel.
    raw1394_write(handle_, node_, 0x1001, 1, (quadlet_t*)&rx_iso_channel_);

    // Not sure what this does, but it is required.
    {
        uint32_t data = 0xf80f0000;
        raw1394_write(handle_, node_, 0x20010, 4, (quadlet_t*)&data);
    }

    // Tell Omni to start isochronous data transmission.
    {
        uint8_t data = 0x08;
        raw1394_write(handle_, node_, 0x1087, 1, (quadlet_t*)&data);
    }

    // Start receiving.
    rx_handle_ = raw1394_new_handle_on_port(port_);
    raw1394_set_userdata(rx_handle_, this);
    raw1394_iso_recv_init(rx_handle_, callbackRead, 20, 0x40,
            rx_iso_channel_, RAW1394_DMA_PACKET_PER_BUFFER, -1);
    raw1394_iso_recv_start(rx_handle_, -1, -1, 0);

    // Start sending.
    tx_handle_ = raw1394_new_handle_on_port(port_);
    raw1394_set_userdata(tx_handle_, this);
    raw1394_iso_xmit_init(tx_handle_, callbackWrite, 1, sizeof(tx_iso_buffer_),
            tx_iso_channel_, RAW1394_ISO_SPEED_100, -1);
    raw1394_iso_xmit_start(tx_handle_, -1, -1);

    // Create iso thread.
    pthread_create(&iso_thread_, NULL, iso_thread_handler, this);
    state_ = RAW_OMNI_STATE_THREAD_STARTED;

    return true;
}

void OmniFirewire::stop_isochronous_transmission()
{
    // Stop the thread.
    if (state_ != RAW_OMNI_STATE_READY) {
        state_ = RAW_OMNI_STATE_THREAD_SHOULD_TERMINATE;
        pthread_join(iso_thread_, NULL);
        state_ = RAW_OMNI_STATE_READY;
    }

    // Make sure raw1394 is ready.
    if (handle_ == NULL) {
        return;
    }

    // Tell Omni to stop isochronous transmission.
    uint8_t data = 0x00;
    raw1394_write(handle_, node_, 0x1087, 1, (quadlet_t*)&data);

    if (tx_iso_channel_ != -1) {
        raw1394_channel_modify(handle_, tx_iso_channel_, RAW1394_MODIFY_FREE);
        tx_iso_channel_ = -1;
    }

    if (rx_iso_channel_ != -1) {
        raw1394_channel_modify(handle_, rx_iso_channel_, RAW1394_MODIFY_FREE);
        rx_iso_channel_ = -1;
    }

    if (tx_handle_ != NULL) {
        raw1394_destroy_handle(tx_handle_);
        tx_handle_ = NULL;
    }

    if (rx_handle_ != NULL) {
        raw1394_destroy_handle(rx_handle_);
        rx_handle_ = NULL;
    }
}

void* OmniFirewire::iso_thread_handler(void* arg)
{
    OmniFirewire* omni = reinterpret_cast<OmniFirewire*>(arg);

    int counter = 0;
    while (omni->state_ != RAW_OMNI_STATE_THREAD_SHOULD_TERMINATE
            && raw1394_loop_iterate(omni->rx_handle_) != -1
            && raw1394_loop_iterate(omni->tx_handle_) != -1) {
        /*
        fputs("\e[H\n\n\n\n\n\n", stdout);

        std::printf("joint angles = ");
        for (size_t i = 0; i < 6; i++) {
            std::printf("% 9.3f", omni->current_joint_angles_[i]);
        }
        std::cout << "\n";

        std::printf(" home angles = ");
        for (size_t i = 0; i < 6; i++) {
            std::printf("% 9.3f", omni->home_joint_angles_[i]);
        }
        std::cout << "\n";

        std::printf("buttons      = %12s%12s\n",
                omni->current_buttons_[0] ? "PRIMARY" : "",
                omni->current_buttons_[1] ? "SECONDARY" : "");

        std::cout << counter++ << std::endl;
        */
    }

    return NULL;
}

std::vector<OmniFirewire::OmniInfo> OmniFirewire::enumerate_omnis()
{
    std::vector<OmniInfo> omnis;
    raw1394handle_t handle = raw1394_new_handle();

    // Get information about the available ports.
    const int maxports = 5;
    struct raw1394_portinfo portinfo[maxports];
    int n_ports = raw1394_get_port_info(handle, portinfo, maxports);

    // Iterate through the list of ports.
    for (int p = 0 ; p < n_ports; p++) {
        // Open the port if we can.
        if (portinfo[p].nodes == 0 || raw1394_set_port(handle, p) == -1) {
            // Not the port we're interested in.
            continue;
        }

        // Iterate through the list of nodes attached to the port.
        for (int n = 0; n < portinfo[p].nodes; n++) {
            // Compute the node ID. See IEEE 1394 spec for details.
            uint32_t node = (1023 << 6) | n;

            // Check the device ID.
            quadlet_t dev_id = 0;
            raw1394_read(handle, node, 0x1006000c, sizeof(dev_id), &dev_id);
            dev_id = ntohl(dev_id);
            if (dev_id != 0x000b9900) {
                // Not Phantom Omni.
                continue;
            }

            // Now we know it's Phantom Omni. Store the info.
            quadlet_t serial = 0;
            raw1394_read(handle, node, 0x10060010, sizeof(serial), &serial);
            const std::string& unpacked_serial
                    = unpack_serial_number(ntohl(serial));
            omnis.push_back(OmniInfo(unpacked_serial, p, node));
        }
    }

    return omnis;
}

void OmniFirewire::firewireMain(int argc, char** argv)
{
    // Initialize ROS.
    ros::init(argc, argv, "raw_omni");

    // Get the omni parameters.
    std::string omni_name;
    ros::param::param<std::string>("~omni_name", omni_name, "");
    std::string omni_serial;
    ros::param::param<std::string>("~omni_serial", omni_serial, "11129400000");

    // List the serial number of each Phantom Omni connected.
    const std::vector<OmniFirewire::OmniInfo>& omnis = OmniFirewire::enumerate_omnis();
    std::ostringstream serial_numbers;
    for (size_t i = 0; i < omnis.size(); i++) {
        serial_numbers << omnis[i].serial << " ";
    }
    ROS_INFO("Phantom Omni Serial Numbers: %s", serial_numbers.str().c_str());

    // Start the node.
    OmniFirewire node(omni_name, omni_serial);
    node.run();
}
