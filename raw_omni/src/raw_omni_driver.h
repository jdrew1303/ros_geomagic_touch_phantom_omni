/*
 * Copyright (c) 2013, 2015 Yutaka Tsutano.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
 * REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT,
 * INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
 * LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
 * OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef RAW_OMNI_DRIVER_H
#define RAW_OMNI_DRIVER_H

#include <vector>
#include <string>
#include <stdint.h>
#include <libraw1394/raw1394.h>
#include <pthread.h>

class RawOmniDriver {
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

public:
    RawOmniDriver(const std::string &serial_number);
    ~RawOmniDriver();
    bool open();

    void get_current_joint_angles(std::vector<double>& angles) {
        // Protect the critical section.
        pthread_mutex_lock(&iso_mutex_);

        angles.clear();
        angles.insert(angles.end(), current_joint_angles_, current_joint_angles_
                + sizeof(current_joint_angles_) / sizeof(*current_joint_angles_));

        pthread_mutex_unlock(&iso_mutex_);
    }
    
    void get_current_button_state(std::vector<bool>& button) {
        // Protect the critical section.
        pthread_mutex_lock(&iso_mutex_);
        
        button.clear();
        button.insert(button.end(), current_buttons_, current_buttons_
                + sizeof(current_buttons_) / sizeof(*current_buttons_));

        pthread_mutex_unlock(&iso_mutex_);
    }

    void enable_force(bool enable) {
    	this->reset_force();

        pthread_mutex_lock(&iso_mutex_);
        
        if (enable)
        {
        	tx_iso_buffer_.status.force_enabled = 1;
        	enable_force_flag = true;
        }
        else
        {
        	tx_iso_buffer_.status.force_enabled = 0;
        	enable_force_flag = false;
        }

        pthread_mutex_unlock(&iso_mutex_);
    }

    void reset_force() {
    	const double force_init [] = {0, 0, 0};
    	this->set_force(force_init);
    }
    
    void set_force(const double * const force) {
        pthread_mutex_lock(&iso_mutex_);
        
        force_output_[0] = (force[0] + 1) / 2 * 4095;
        force_output_[1] = (force[1] + 1) / 2 * 4095;
        force_output_[2] = (force[2] + 1) / 2 * 4095;
                        
        pthread_mutex_unlock(&iso_mutex_);
    }    
    
    bool calibrated() { return calibrated_; }
    const std::string& serial_number() { return serial_number_; }
    void close();
    bool opened() { return handle_ != NULL; }
    static std::vector<OmniInfo> enumerate_omnis();
};

#endif
