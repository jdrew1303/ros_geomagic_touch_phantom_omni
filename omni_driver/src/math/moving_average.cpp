#include "math/moving_average.hpp"

MovingAverage::MovingAverage(const int filter_size, const int input_size)
    : filter_size(filter_size), input_size(input_size) {
    data.resize(filter_size, std::vector<double>(input_size, 0.0));
}
