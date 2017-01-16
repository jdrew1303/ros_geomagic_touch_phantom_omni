#include "omnibase.h"

#include <iostream>


OmniBase::OmniBase()
{
}


void OmniBase::driverCallback(void *data)
{
    OmniState * state = static_cast<OmniState *>(data);
    if (!state)
    {
        std::cerr << "Could not cast the data passed from the driver. The robot will not work." << std::endl;
        return;
    }

    // Acquire a unique lock to prevent others from writing
    LockUnique lock( getStateMutex() );

    // Call the implementation specific callback
    callback(state);

    // Store the current time
    Time time_now( Clock::local_time() );

    // If this is not the first callback call, update the velocity
    if (state->seq > 0)
    {
        TimeDuration dt_boost = time_now - state->stamp;
        double dt = ( (double) dt_boost.total_microseconds() ) / 1e+6;
        for (int k = 0; k < state->angles.size(); ++k)
        {
            state->velocities[k] = (state->angles[k] - state->angles_last[k]) / dt;
        }
    }

    // Compute the forward kinematics - TODO

    // Prepare for the next call
    state->angles_last = state->angles;
    state->stamp = time_now;
    state->seq++;
}
