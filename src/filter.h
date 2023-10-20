// Horribly lame digital filter code

// Currently implements a static IIR filter.

#include "eigen.h"

template<typename T, unsigned C>
class xir_filter {
    typedef Eigen::Array<T, C, 1> array_t;
    const array_t a_coeff, b_coeff;
    const T gain;
    array_t x, y;

    public:
    xir_filter(T gain_, array_t a, array_t b) : gain(gain_), a_coeff(std::move(a)), b_coeff(std::move(b)), x(array_t::Zero()), y(array_t::Zero()) {};

    T operator()(T input) {
        x.head(C-1) = x.tail(C-1);  // shift by one -- does this work?
        x(C-1) = input / gain;
        y.head(C-1) = y.tail(C-1);  // shift by one -- does this work?
        y(C-1) = (x * b_coeff).sum() - (y.head(C-1) * a_coeff.head(C-1)).sum();
        return y(C-1);
    }
};
