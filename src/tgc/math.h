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
#ifndef TGC_MATH_H_
#define TGC_MATH_H_

#include <limits>
#include <cmath>

namespace tgc {
    
constexpr double kInfty = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/**
 * @brief Computes the value of $arccos(b - (ax)^2)$; is more stable for small $x$.
 */
inline double acos_abxx(double b, double a, double x) {
    return 2 * asin(sqrt(
        (1 - b + a*x*x) / 2
    ));
}

} // namespace tgc

#endif // TGC_MATH_H_