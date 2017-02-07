#pragma once

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


class OmniEthernet : public OmniBase
{
private:
    void getJointAnglesFromDriver();
    void autoCalibration();
    void forceCallback(const omni_driver::OmniFeedback::ConstPtr& omnifeed);
    static HDCallbackCode HDCALLBACK callback(void *pdata);
    HDErrorInfo error;
    HHD hHD;

public:
    static int calibrationStyle;
    OmniEthernet(const std::string &name = "");
    bool connect();
    bool connected();
    void disconnect();
};

typedef boost::shared_ptr<OmniEthernet> OmniEthernetPtr;
