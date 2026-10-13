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
#ifndef TGC_TWIST_GAIT_H_
#define TGC_TWIST_GAIT_H_

#include <memory>
#include <vector>
#include <limits>

#include "bolt/eigen/dense.h"
#include "bolt/base/option.h"
#include "bolt/base/result.h"

#include "tgc/shaping_functions.h"
#include "tgc/math.h"

namespace tgc {

class TwistGait {

    struct {
        ShapingFunction          a;
        ShapingFunction          b;
        ShapingFunction          c;
        bolt::Mat<-1>            C;
        double                   tau_swing;
        double                   h_swing;   
        double                   r_ws;
        std::vector<bolt::SE3<>> T0;

        double duty_factor;

        std::vector<bolt::SE3<>>          T_tip;
        std::vector<bolt::Vec<6>>         V_tip;
        std::vector<bolt::Option<double>> phi;

        // bool is_V_updated = false;
    } _self;

    template<typename = void, typename... Args>
    explicit TwistGait(Args&&... args) : _self{std::move<Args>(args)...} {
        reset();
    }
public:
    enum class eError {
        InvalidGaitCorrelations,
    };

    static bolt::Result<TwistGait, eError>
    make(
        ShapingFunction a,
        ShapingFunction b,
        ShapingFunction c,
        bolt::MatCRef<> C,
        double tau_swing,
        double h_swing,
        double r_ws,
        const std::vector<bolt::SE3<>>& T0
    );

    inline size_t nLegs() const { return _self.T0.size(); }

    void reset();
    void step(bolt::VecCRef<6> V, double delta);

    inline const std::vector<bolt::SE3<>> T_tip() const {
        return _self.T_tip;
    }

    inline void unsafe_set_gait_correlations(bolt::MatCRef<> new_G) {
        double row_sum = new_G.row(0).sum();
        int n = new_G.cols();

        _self.duty_factor = (n - row_sum)/(n + row_sum);
        _self.C = new_G;
    }

    // Force a leg to be in the stance phase
    inline void force_stance(size_t leg_idx) {
        _self.phi[leg_idx] = bolt::None;
    }

    // Get the phase of a leg (None if in stance, Some(phase) if in swing)
    inline bolt::Option<double> phase(size_t leg_idx) const {
        return _self.phi[leg_idx];
    }

    // Returns true if the leg at the given index is in the stance phase.
    inline bool is_stance(size_t leg_idx) const {
        return !_self.phi[leg_idx].is_some();
    }
};

} // namespace tgc

#endif // TGC_TWIST_GAIT_H_
