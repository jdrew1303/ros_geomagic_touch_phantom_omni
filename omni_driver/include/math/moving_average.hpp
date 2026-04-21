#pragma once

#include <vector>
#include <stdexcept>

class MovingAverage {
    private:

    int input_counter = 0;
    
    std::vector<std::vector<double>> data;
    
    public: 

    const int filter_size;
    const int input_size;

    MovingAverage(const int filter_size, const int input_size);

    std::vector<double> mean() {
        std::vector<double> m(input_size);
        for (int i = 0; i < input_size; i++) {
            for (int j = 0; j < input_counter; j++) {
                m[i] += data[j][i];
            }
            if (input_counter > 0) m[i] = m[i]/input_counter; 
        }
        return m;
    }

    void input(std::vector<double> input_data) {
        if (input_data.size() != (size_t)input_size) throw std::logic_error("Input size must be fixed!.");
        data.insert(data.begin(), input_data);
        data.resize(filter_size);
        if (input_counter < filter_size) input_counter++;
    }
};
