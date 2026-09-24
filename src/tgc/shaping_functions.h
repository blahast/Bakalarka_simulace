/**
 * @file
 * @brief 
 * 
 * @author David Valouch (valouch.david@fel.cvut.cz)
 * @date 2025-09-05
 * 
 * @copyright Copyright (c) 2025
 */

#pragma once
#ifndef TGC_SHAPING_FUNCTIONS_H_
#define TGC_SHAPING_FUNCTIONS_H_

#include <cmath>
#include <functional>

namespace tgc {

struct ShapingFunction {
    std::function<double(double)> f;
    std::function<double(double)> F;

    inline double operator() (double phi) const {
        return f(phi);
    }
    inline double I(double lb, double ub) const {
        return F(ub) - F(lb);
    };
};

inline ShapingFunction StrideCos() {
    return ShapingFunction{
        [](double phi) -> double { return cos(2*M_PI*phi); },
        [](double phi) -> double { return sin(2*M_PI*phi)/2/M_PI; }
    };
}

inline ShapingFunction LiftSin() {
    return ShapingFunction{
        [](double phi) -> double { return M_PI * sin(2*M_PI*phi); },
        [](double phi) -> double { return -cos(2*M_PI*phi)/2; }
    };
}

inline ShapingFunction GainSin() {
    return ShapingFunction{
        [](double phi) -> double { return M_PI_2 * sin(M_PI*phi); },
        [](double phi) -> double { return M_PI_2 * -cos(2*M_PI*phi)/M_PI; }
    };
}

inline ShapingFunction GainCos() {
    return ShapingFunction{
        [](double phi) -> double { return 1. - cos(2*M_PI*phi); },
        [](double phi) -> double { return phi - sin(2*M_PI*phi)/2/M_PI; }
    };
}
    
} // namespace tgc

#endif // TGC_SHAPING_FUNCTIONS_H_