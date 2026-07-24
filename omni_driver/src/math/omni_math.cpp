#include "math/omni_math.hpp"

OmniMath::OmniMath(const bool enable_moving_average)
    : moving_average(5, 6), enable_moving_average(enable_moving_average) {
}
