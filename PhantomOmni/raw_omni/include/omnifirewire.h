#pragma once

#include "omnibase.h"
#include "ros/ros.h
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
#include <pthread.h>

#define RAW_OMNI_POT_FILTER_TAPS 20

class OmniFirewire : public OmniBase
{
// OmniBase interface
protected:
    void callback(OmniState *state);
    enum raw1394_iso_disposition callbackWrite(raw1394handle_t handle,unsigned char* data, unsigned int* len, unsigned char* tag, unsigned char* sy, int cycle, unsigned int dropped);
    enum raw1394_iso_disposition callbackRead(raw1394handle_t handle, unsigned char* data, unsigned int len, unsigned char channel, unsigned char tag, unsigned char sy, unsigned int cycle, unsigned int dropped);
    void* iso_thread_handler(void* arg);
    std::vector<RawOmniDriver::OmniInfo> OmniFirewire::enumerate_omnis();
public:
    bool connect();
    void disconnect();
    OmniFirewire(const std::string &serial_number);
    ~OmniFirewire();
    void timerHandler(const ros::TimerEvent& event);
    bool run();
    void firewireMain(int argc, char** argv);
    uint32_t pack_serial_number(const std::string& unpacked);
    std::string unpack_serial_number(uint32_t packed);
    bool RawOmniDriver::start_isochronous_transmission();
    bool RawOmniDriver::stop_isochronous_transmission();

private:
    struct TxIsoBuffer {
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

    struct RxIsoBuffer {
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

private:
    double pot_filter_accum_[3];
    int pot_filter_count_;
    bool enable_force_flag;
    double current_joint_angles_[6];
    double home_joint_angles_[6];
    int16_t force_output_[3];
    bool current_buttons_[2];
    bool calibrated_;

    pthread_t iso_thread_;
    pthread_mutex_t iso_mutex_;
    volatile enum {
        RAW_OMNI_STATE_READY,
        RAW_OMNI_STATE_THREAD_STARTED,
        RAW_OMNI_STATE_THREAD_SHOULD_TERMINATE
    } state_;

    std::string serial_number_;
    raw1394handle_t handle_, tx_handle_, rx_handle_;
    int32_t port_, node_;
    int8_t tx_iso_channel_;
    int8_t rx_iso_channel_;

    static uint32_t pack_serial_number(const std::string& unpacked);
    static std::string unpack_serial_number(uint32_t packed);

    bool start_isochronous_transmission();
    void stop_isochronous_transmission();

    static void* iso_thread_handler(void* arg);

    static enum raw1394_iso_disposition txHandler(raw1394handle_t handle,
            unsigned char* data, unsigned int* len,
            unsigned char* tag, unsigned char* sy, int cycle,
            unsigned int dropped);
    static enum raw1394_iso_disposition rxHandler(raw1394handle_t handle,
        unsigned char *data, unsigned int len, unsigned char channel,
        unsigned char tag, unsigned char sy, unsigned int cycle,
        unsigned int dropped);

public:
    struct OmniInfo {
        std::string serial;
        int32_t port;
        int32_t node;
        OmniInfo(const std::string& serial, int32_t port, int32_t node)
                : serial(serial), port(port), node(node) {}
    };
};
