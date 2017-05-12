#pragma once


#include "omnibase.h"
#include "ros/ros.h"
#include "raw1394msg.h"


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
#include <math.h>


class OmniFirewire : public OmniBase
{
private:
    int rx_handle_status, tx_handle_status;
    static const unsigned int GIMBAL_FILTER_SIZE = 20;

    std::vector<double> gimbal_filter_1;
    std::vector<double> gimbal_filter_2;
    std::vector<double> gimbal_filter_3;

    typedef boost::shared_ptr<boost::thread> ThreadPtr;
    int pot_filter_count_;
    double pot_filter_accum_[3];
    std::string serial_number_;
    ThreadPtr thread_driver;
    int16_t force_output_[3];
    int32_t port_, node_;
    raw1394handle_t handle_, tx_handle_, rx_handle_;

    // raw1394 messages
    Raw1394Msg msg_rx_iso_channel;
    Raw1394Msg msg_tx_iso_channel;
    Raw1394Msg msg_start;
    Raw1394Msg msg_stop;
    Raw1394Msg msg_unknown;


    volatile enum THREAD_STATE                          ///< Thread states.
    {
        THREAD_READY,
        THREAD_STARTED,
        THREAD_TERMINATE
    } thread_status;

    struct TxIsoBuffer     ///< Struct used for writing mainly to omni motors.
    {
        int16_t force_x;
        int16_t force_y;
        int16_t force_z;

        union {
            uint16_t bits;
            struct {
                bool dock_led0     : 1;
                bool dock_led1     : 1;
                bool bit2          : 1;
                bool force_enabled : 1;
                bool bit4          : 1;
                bool bit5          : 1;
                bool bit6          : 1;
                bool bit7          : 1;
                bool bit8          : 1;
                bool bit9          : 1;
                bool bit10         : 1;
                bool bit11         : 1;
                bool bit12         : 1;
                bool bit13         : 1;
                bool bit14         : 1;
                bool bit15         : 1;
            };
        } status;

        uint32_t padding1;
        uint32_t padding2;
    } tx_iso_buffer_;

    struct RxIsoBuffer     ///< Struct used for reading data from the omni device.
    {
        uint32_t magic;

        int16_t encoder_x;
        int16_t encoder_y;
        int16_t encoder_z;

        uint16_t gimbal_a_x;
        uint16_t gimbal_a_y;
        uint16_t gimbal_a_z;

        uint16_t v0;
        uint8_t tx_packet_num;

        union {
            uint8_t bits;
            struct {
                bool button1  : 1;
                bool button2  : 1;
                bool undocked : 1;
                bool bit3     : 1;
                bool bit4     : 1;
                bool bit5     : 1;
                bool bit6     : 1;
                bool bit7     : 1;
            };
        } status;

        uint16_t s0;

        uint16_t gimbal_b_x;
        uint16_t gimbal_b_y;
        uint16_t gimbal_b_z;

        uint16_t v1;
        uint16_t s1;

        uint32_t time;

        uint16_t t0;
        uint16_t t1;

        uint32_t wall_time;
    };

    /**
     * @brief Used to pack the omni serial number.
     * @param unpacked Unpacked serial number in string format.
     * @return Returns packed serial number in uint32_t format.
     */
    uint32_t pack_serial_number(const std::string& unpacked);

    /**
     * @brief Used to unpack omni serial number.
     * @param packed Packed serial number in uint32_t format.
     * @return Returns unpacked serial number in string format.
     * @see packSerialNumber
     */
    static std::string unpackSerialNumber(uint32_t packed);

    /**
     * @brief The callback used by the driver to write data to omni.
     * @param handle
     * @param data
     * @param len
     * @param tag
     * @param sy
     * @param cycle
     * @param dropped
     * @return Returns a raw1394_iso_disposition to be used by raw1394_iso_xmit_init().
     */
    static enum raw1394_iso_disposition callbackWrite(raw1394handle_t handle,unsigned char* data, unsigned int* len, unsigned char* tag, unsigned char* sy, int cycle, unsigned int dropped);

    /**
     * @brief The callback used by the driver to read data from omni.
     * @param handle
     * @param data
     * @param len
     * @param tag
     * @param sy
     * @param cycle
     * @param dropped
     * @return Returns a raw1394_iso_disposition to be used by raw1394_iso_recv_init().
     */
    static enum raw1394_iso_disposition callbackRead(raw1394handle_t handle, unsigned char* data, unsigned int len, unsigned char channel, unsigned char tag, unsigned char sy, unsigned int cycle, unsigned int dropped);

    /**
     * @brief The callback that will be called routinely by the driver.
     */
    void * isoThreadCallback();

    /**
     * @brief Used to check if omni device is connected.
     * @return Returns 1 if connected, 0 otherwise.
     */
    bool connected();

    /**
     * @brief Wakes up.
     */
    void wakeup();

    /**
     * @brief Starts the firewire isochronous transmission.
     * @return Returns 1 if succesfull, 0 otherwise.
     */
    bool startIsochronousTransmission();

    /**
     * @brief Stops the isochronous transmission.
     */
    void stopIsochronousTransmission();

protected:
    void mapTorque();

public:

    struct OmniInfo                         ///< Structure with device connection information.
    {
        std::string serial;
        int32_t port;
        int32_t node;

        OmniInfo(const std::string& serial, int32_t port, int32_t node)
                : serial(serial), port(port), node(node) {}
    };


    /**
     * @brief Used to connect with omni device.
     * @return Returns 1 if successfull, 0 otherwise.
     */
    bool connect();

    /**
     * @brief Used to disconnect with omni device.
     */
    void disconnect();

    /**
     * @brief Used to enumerate devices found.
     * @return Returns a vector with information from different devices in the OmniInfo struct format.
     */
    static std::vector<OmniFirewire::OmniInfo> enumerate_omnis();

    /**
     * @brief OmniFirewire constructor, only sets some members needed for communicating with omni.
     * @param serial_number Reference to string of serial number.
     * @param name Reference to string of omni name.
     */
    OmniFirewire(const std::string &serial_number, const std::string &name, const std::string &path_urdf, const std::string &path_srdf);
    ~OmniFirewire();

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
};

typedef boost::shared_ptr<OmniFirewire> OmniFirewirePtr;
