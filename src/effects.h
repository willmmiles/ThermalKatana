// Effect definitions

#include <array>
#include <cstdint>
#include <eigen.h>
#include "xir_filter.h"

class wave {
    static constexpr std::uint16_t wave_values[] = { 1, 1, 2, 3, 4, 5, 4, 3, 2, 1, 1 };
    size_t index;

    public:
    wave() : index(0) {};

    template<typename data_t, unsigned size>
    void operator()(Eigen::Array<data_t, size, 1> & lights, uint16_t scale) {
        auto map = Eigen::Map<const Eigen::Array<uint16_t, 11, 1>>(wave_values);
        if ((index + map.size()) < lights.size()) {
            lights.segment(index, map.size()) += (map * scale);
        } else {
            // two part
            auto split = lights.size() - index;
            lights.segment(index, split) += map.segment(0, split) * scale;
            lights.segment(0, map.size() - split) += map.segment(split, map.size() - split) * scale;
        }

        index = (index + 1) % lights.size();
    }  
};

template<unsigned size>
class energy {
    typedef Eigen::Array<int16_t, size, 1> state_t;
    state_t state;
    size_t index;

    public:
    energy() : state(state_t::Zero()), index(0) {};

    void operator()(Eigen::Array<uint16_t, size, 1> & lights, float energy, int shift = 1) {
        state[index] = (int16_t) energy;
        auto split = state.size() - index;
        lights.segment(0, split) += state.segment(index, split).template cast<uint16_t>();
        lights.segment(split, index) += state.segment(0, index).template cast<uint16_t>();

        index = (index + size - shift) % size;
    }  
};


// Apply a filter with a cap - returns a scale from 0 to 1
class surge {
    xir_filter<float, 3> filter;
    
    public:
    // 20Hz butterworth
    surge() : filter(14.824637753965225, {0.41280159809618855,-1.142980502539901,1}, {1,2,1}) {};

    float operator()(float energy) {
        if (energy < 0) energy = 0;
        else if (energy > 200) energy = 200;
        return std::min(filter(energy), 256.f) / 256.f;
    }
};


template<unsigned size>
class sparkle {
    static constexpr int pixel_count[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2 };
    typedef Eigen::Array<uint16_t, size, 1> state_t;
    state_t state;

    public:
    sparkle() : state(state_t::Zero()) {};

    template<typename data_t>
    void operator()(Eigen::Array<data_t, size, 1> & lights) {
        state /= 5;
        auto n_pixels = random(sizeof(pixel_count) / sizeof(int));
        for (int i = 0; i < pixel_count[n_pixels]; i++) {
            auto j = random(NUM_LEDS);
            state[j] = 120;
        }

        lights += state.template cast<data_t>();
    }
};

