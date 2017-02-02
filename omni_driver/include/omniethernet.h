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
#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HD/hdExport.h>

class OmniEthernet : public OmniBase
{
private:
    void getJointAnglesFromDriver(OmniState *state);
    void autoCalibration();
    void forceCallback(const omni_driver::OmniFeedbackConstPtr& omnifeed);
    static HDCallbackCode HDCALLBACK HDCallbackCode(void *data);

public:
    OmniEthernet(const std::string &name = "");
    bool connect();
    bool connected();
    void disconnect();
};
