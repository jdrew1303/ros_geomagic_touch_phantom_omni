#pragma once

#include "omnibase.h"
#include "raw_omni_driver.h"

class OmniFirewire : public OmniBase
{
// OmniBase interface
protected:
    void callback(OmniState *state);

public:
    bool connect();
    bool disconnect();
    OmniFirewire();
};
