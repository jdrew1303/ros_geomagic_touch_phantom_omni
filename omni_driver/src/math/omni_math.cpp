#include "math/omni_math.hpp"

MovingAverage::MovingAverage(const int filter_size, const int input_size)
    : filter_size(filter_size), input_size(input_size) {
    data.resize(filter_size, std::vector<double>(input_size, 0.0));
}

OmniMath::OmniMath(const bool enable_moving_average)
    : moving_average(5, 6), enable_moving_average(enable_moving_average) {
}
