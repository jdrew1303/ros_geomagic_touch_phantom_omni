#pragma once


#include "omnibase.h"


class OmniFirewire : public OmniBase
{
public:
    OmniFirewire();

// OmniBase interface
protected:
    void callback(OmniState *state);
public:
    bool connect();
    bool disconnect();
};
