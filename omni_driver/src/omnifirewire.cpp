#include "../include/omnifirewire.h"

#include <numeric>

#include <boost/bind.hpp>


OmniFirewire::OmniFirewire(const std::string &serial_number, const std::string &name, const std::string &path_urdf, const std::string &path_srdf)
    : OmniBase(name, path_urdf, path_srdf),
      gimbal_filter_1(GIMBAL_FILTER_SIZE),
      gimbal_filter_2(GIMBAL_FILTER_SIZE),
      gimbal_filter_3(GIMBAL_FILTER_SIZE),
      serial_number_(serial_number),
      port_(-1), node_(-1),
      handle_(NULL), tx_handle_(NULL), rx_handle_(NULL),
      msg_rx_iso_channel(0x1001, 1, -1),
      msg_tx_iso_channel(0x1000, 1, -1),
      msg_start(0x1087, 1, 0x08),
      msg_stop(0x1087, 1, 0x00),
      msg_unknown(0x20010, 4, 0xf80f0000),
     thread_status(THREAD_READY)
{
    this->resetTorque();
}

OmniFirewire::~OmniFirewire()
{
    this->disconnect();
}

enum raw1394_iso_disposition OmniFirewire::callbackRead(raw1394handle_t handle, unsigned char* data, unsigned int len, unsigned char channel, unsigned char tag, unsigned char sy, unsigned int cycle, unsigned int dropped)
{
    // Get the OmniFirewire object for this handle.
    OmniFirewire* omni = reinterpret_cast<OmniFirewire*>(raw1394_get_userdata(handle));
    if (omni == NULL)
    {
        return RAW1394_ISO_OK;
    }

    const OmniFirewire::RxIsoBuffer* newbuf = reinterpret_cast<OmniFirewire::RxIsoBuffer*>(data);

    OmniBase::LockUnique lock( omni->getStateMutex() );

    // Calibrate if docked.
    if (!newbuf->status.undocked && !omni->state.calibrated)
    {
        ROS_INFO("Calibrating robot...");

        // Calibrate.
        // All angles should be ~= 0 when docked.
        omni->state.angles_docked[0] = 2.0 * M_PI *  newbuf->encoder_x / 15000.0;
        omni->state.angles_docked[1] = 2.0 * M_PI * -newbuf->encoder_y / 15000.0;
        omni->state.angles_docked[2] = 2.0 * M_PI *  newbuf->encoder_z / 15000.0 + 0.37;
        // Calibrated: the dock LED should be always on.
        omni->tx_iso_buffer_.status.dock_led0 = 1;
        omni->tx_iso_buffer_.status.dock_led1 = 1;
        omni->state.calibrated = true;
    }

    // Getting time difference between two consecutive readings.
//    omni->state.time_last_angle_acquisition = omni->state.time_current_angle_acquisition;
    omni->state.time_current_angle_acquisition = boost::posix_time::microsec_clock::local_time();

    // Compute the encoder values.
    // All angles should be ~= 0 when docked.
    omni->state.angles[0] = 2.0 * M_PI
            *  newbuf->encoder_x / 15000.0 - omni->state.angles_docked[0];

    omni->state.angles[1] = 2.0 * M_PI
            * -newbuf->encoder_y / 15000.0 - omni->state.angles_docked[1];

    omni->state.angles[2] = 2.0 * M_PI
            *  newbuf->encoder_z / 15000.0 - omni->state.angles_docked[2]
            - omni->state.angles[1] + 0.37;

    // Adding values to the filter vector.
    std::rotate(omni->gimbal_filter_1.begin(),
                omni->gimbal_filter_1.begin() + 1,
                omni->gimbal_filter_1.end());
    omni->gimbal_filter_1[GIMBAL_FILTER_SIZE - 1] = (double)newbuf->gimbal_a_x
            / (newbuf->gimbal_a_x + newbuf->gimbal_b_x);
    //
    std::rotate(omni->gimbal_filter_2.begin(),
                omni->gimbal_filter_2.begin() + 1,
                omni->gimbal_filter_2.end());
    omni->gimbal_filter_2[GIMBAL_FILTER_SIZE - 1] = (double)newbuf->gimbal_a_y
            / (newbuf->gimbal_a_y + newbuf->gimbal_b_y);
    //
    std::rotate(omni->gimbal_filter_3.begin(),
                omni->gimbal_filter_3.begin() + 1,
                omni->gimbal_filter_3.end());
    omni->gimbal_filter_3[GIMBAL_FILTER_SIZE - 1] = (double)newbuf->gimbal_a_z
            / (newbuf->gimbal_a_z + newbuf->gimbal_b_z);

    if (omni->pot_filter_count_ <= 0)
    {
        // Compute the mean.
        omni->pot_filter_accum_[0] =
                std::accumulate(omni->gimbal_filter_1.begin(), omni->gimbal_filter_1.end(), 0.0)
                / GIMBAL_FILTER_SIZE;
        omni->pot_filter_accum_[1] =
                std::accumulate(omni->gimbal_filter_2.begin(), omni->gimbal_filter_2.end(), 0.0)
                / GIMBAL_FILTER_SIZE;
        omni->pot_filter_accum_[2] =
                std::accumulate(omni->gimbal_filter_3.begin(), omni->gimbal_filter_3.end(), 0.0)
                / GIMBAL_FILTER_SIZE;

        // Compute the gimbal values.
        // All angles should be ~= 0 when docked.
        omni->state.angles[3] = -(5.48 * omni->pot_filter_accum_[0] - 2.74) + 0.020 + 0.025367101673835286;
        omni->state.angles[4] = -(5.28 * 1.9 * omni->pot_filter_accum_[1] + 2.64) + 8.76231837158503;
        omni->state.angles[5] =  (4.76 * omni->pot_filter_accum_[2] - 2.3) - 0.085;

    }
    else
    {
        --omni->pot_filter_count_;
    }

    // Calculates forward kinematics and joint velocities.
    omni->updateRobotState();

    // Get the button state.
    omni->state.buttons[0] = !newbuf->status.button1;
    omni->state.buttons[1] = !newbuf->status.button2;

    return RAW1394_ISO_OK;
}

enum raw1394_iso_disposition OmniFirewire::callbackWrite(raw1394handle_t handle, unsigned char *data, unsigned int* len, unsigned char *tag, unsigned char *sy, int, unsigned int)
{
    *len = 0;
    *tag = 0;
    *sy = 0;
    // Get the OmniFirewire object for this handle.
    OmniFirewire *omni = reinterpret_cast<OmniFirewire*>(raw1394_get_userdata(handle));
    if (omni == NULL)
    {
        return RAW1394_ISO_OK;
    }

    // Edit force output
    {
        OmniBase::LockShared lock( omni->getStateMutex() );
        if (omni->state.control_on)
        {
            omni->tx_iso_buffer_.force_x = omni->state.control[0];
            omni->tx_iso_buffer_.force_y = omni->state.control[1];
            omni->tx_iso_buffer_.force_z = omni->state.control[2];
        }
        else
        {
            omni->tx_iso_buffer_.status.force_enabled = 0;
        }
    }

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



bool OmniFirewire::connect()
{
    // Make sure the device is disconnected at this point.
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
    pot_filter_count_ = GIMBAL_FILTER_SIZE;

    // Start isochronous transmission.
    if (!startIsochronousTransmission()) {
        ROS_ERROR("Failed to start isochronous transmission!");
        return false;
    }
    state.connected = true;
    ROS_INFO("Isochronous transmission started successfully.");
    return true;
}

bool OmniFirewire::connected()
{
    return this->state.connected;
}

void OmniFirewire::wakeup()
{
    raw1394_wake_up(rx_handle_);
    raw1394_wake_up(tx_handle_);
    this->disconnect();
    this->connect();
}

void OmniFirewire::disconnect()
{
    // Turn the dock LED off.
    tx_iso_buffer_.status.dock_led0 = 0;
    tx_iso_buffer_.status.dock_led1 = 0;

    {
        OmniBase::LockUnique lock( getStateMutex() );
        this->state.calibrated = false;
    }

    // Stop isochronous data transmission.
    stopIsochronousTransmission();

    port_ = -1;
    node_ = -1;

    if ( connected() )
    {
        raw1394_destroy_handle(handle_);
        handle_ = NULL;
    }

}

std::vector<OmniFirewire::OmniInfo> OmniFirewire::enumerate_omnis()
{
    std::vector<OmniInfo> omnis;
    raw1394handle_t handle = raw1394_new_handle();

    // Get information about the available ports.
    const int maxports = 5;
    struct raw1394_portinfo portinfo[maxports];
    int n_ports = raw1394_get_port_info(handle, portinfo, maxports);

    // Set messages
    Raw1394Msg msg_dev_id = Raw1394Msg(0x1006000c, sizeof(quadlet_t));
    Raw1394Msg msg_serial = Raw1394Msg(0x10060010, sizeof(quadlet_t));

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
            msg_dev_id.read(handle, node);
            if (msg_dev_id.getData() != 0x000b9900) {
                // Not Phantom Omni.
                continue;
            }

            // Now we know it's Phantom Omni. Store the info.
            msg_serial.read(handle, node);
            const std::string &unpacked_serial = OmniFirewire::unpackSerialNumber(msg_serial);
            omnis.push_back(OmniInfo(unpacked_serial, p, node));
        }
    }

    return omnis;
}

void * OmniFirewire::isoThreadCallback()
{
    this->rx_handle_status = 0;
    this->tx_handle_status = 0;
    while (this->thread_status != THREAD_TERMINATE
           && this->rx_handle_status != -1
           && this->tx_handle_status != -1)
    {
        this->rx_handle_status = raw1394_loop_iterate(this->rx_handle_);
        this->tx_handle_status = raw1394_loop_iterate(this->tx_handle_);
    }

    this->state.connected = false;

    return NULL;
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


bool OmniFirewire::startIsochronousTransmission()
{
    // Make sure raw1394 is ready.
    if (handle_ == NULL) {
        return false;
    }

    // Stop previous transmission.
    stopIsochronousTransmission();

    // Find a free isochronous channel number for Tx.
    for (int ch = 0; ch < 63; ch++) {
        if (raw1394_channel_modify(handle_, ch, RAW1394_MODIFY_ALLOC) == 0) {
            msg_tx_iso_channel = ch;
            break;
        }
    }
    if (msg_tx_iso_channel == (quadlet_t) -1) {
        // Channel not found. Abort.
        stopIsochronousTransmission();
        return false;
    }

    // Set the Tx isochronous channel.
    msg_tx_iso_channel.write(handle_, node_);

    // Set the Rx isochronous channel.
    msg_rx_iso_channel.write(handle_, node_);

    // Not sure what this does, but it is required.
    msg_unknown.write(handle_, node_);

    // Tell Omni to start isochronous data transmission.
    msg_start.write(handle_, node_);

    // Register the read (rx) callback that will be called by the driver
    rx_handle_ = raw1394_new_handle_on_port(port_);
    raw1394_set_userdata(rx_handle_, this);
    if (raw1394_iso_recv_init(rx_handle_,
                              OmniFirewire::callbackRead,
                              100,
                              0x40,
                              msg_rx_iso_channel,
                              RAW1394_DMA_PACKET_PER_BUFFER,
                             -1) == -1)
    {
        stopIsochronousTransmission();
        return false;
    }

    if (raw1394_iso_recv_start(rx_handle_, -1, -1, 0) == -1)
    {
        stopIsochronousTransmission();
        return false;
    }

    // Register the write (tx) callback that will be called by the driver
    tx_handle_ = raw1394_new_handle_on_port(port_);
    raw1394_set_userdata(tx_handle_, this);
    if (raw1394_iso_xmit_init(tx_handle_,
                              OmniFirewire::callbackWrite,
                              1,
                              sizeof(tx_iso_buffer_),
                              msg_tx_iso_channel,
                              RAW1394_ISO_SPEED_100,
                             -1) == -1)
    {
        stopIsochronousTransmission();
        return false;
    }

    if (raw1394_iso_xmit_start(tx_handle_, -1, -1) == -1)
    {
        stopIsochronousTransmission();
        return false;
    }

    // Create iso thread
    thread_driver = ThreadPtr( new boost::thread( boost::bind(&OmniFirewire::isoThreadCallback, this) ) );
    thread_status = THREAD_STARTED;

    return true;
}

void OmniFirewire::stopIsochronousTransmission()
{
    // Stop the thread.
    if (thread_status != THREAD_READY) {
        thread_status = THREAD_TERMINATE;

        // Make sure any pending calls are destroyed
        raw1394_wake_up(rx_handle_);
        raw1394_wake_up(tx_handle_);

        thread_driver->join();
    }
    thread_status = THREAD_READY;

    // Make sure raw1394 is ready.
    if (handle_ == NULL) {
        return;
    }

    // Tell Omni to stop isochronous transmission.
    msg_stop.write(handle_, node_);

    if (msg_tx_iso_channel != (quadlet_t) -1) {
        raw1394_channel_modify(handle_, msg_tx_iso_channel, RAW1394_MODIFY_FREE);
        msg_tx_iso_channel.clearData();
    }

    if (msg_rx_iso_channel != (quadlet_t) -1) {
        raw1394_channel_modify(handle_, msg_rx_iso_channel, RAW1394_MODIFY_FREE);
        msg_rx_iso_channel.clearData();
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

void OmniFirewire::mapTorque()
{
    state.control[0] = (state.control[0] + 1) / 2 * 4095;
    state.control[1] = (state.control[1] + 1) / 2 * 4095;
    state.control[2] = (state.control[2] + 1) / 2 * 4095;
}

std::string OmniFirewire::unpackSerialNumber(uint32_t packed)
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
