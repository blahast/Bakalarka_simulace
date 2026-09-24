/**
 * @file
 * @brief 
 * 
 * @author David Valouch (valouch.david@fel.cvut.cz)
 * @date 2025-03-12
 * 
 * @copyright Copyright (c) 2025
 */

#pragma once
#ifndef TGC_TWIST_GAIT_POSITIONAL_H_
#define TGC_TWIST_GAIT_POSITIONAL_H_

#include "tgc/twist_gait.h"

#include "bolt/base/option.h"
#include "bolt/kinematics/kinematic_chain.h"

#include "yaml-cpp/yaml.h"

#include <utility>
#include <string>
#include <vector>

namespace tgc {

/**
 * @brief 
 */
class TwistGait_Positional {
    struct {
        bolt::KinematicChain          leg_model;
        std::vector<bolt::SE3Store<>> leg_Tb;
        std::vector<std::string>      leg_names;
        bolt::VecStore<>              nominal_leg_q;
        std::vector<std::string>      leg_joint_names;
        double                        ws_r;

        double                        step_h;
        double                        t_stance;
        double                        t_swing;
        bolt::VecStore<>              leg_gait_offset;
        bolt::MatStore<>              G;    // Gait correlations

        TwistGait gait;

        // State --------------------------------------------------------------
        bolt::VecStore<>                  q;
    } _;

    template<typename... Args>
    explicit TwistGait_Positional(Args&&... args) : _{std::move<Args>(args)...} {
        reset();
    }
public:
    static bolt::Option<TwistGait_Positional> make(const YAML::Node& yml_data);
    void printLegs() const;
    std::vector<std::string> jointNames() const;

    inline int nLegs() const { return _.leg_Tb.size(); }
    inline int nDof() const { return nLegs() * _.leg_model.nDof(); }

    void reset();

    bolt::Vec<> nominal_q() const;
    bolt::Vec<> next_q(
        bolt::VecCRef<6> V,
        bolt::VecCRef<>  q,
        double           t,
        double           dt);

    inline const bolt::MatStore<>& gait_correlations() const {
        return _.G;
    }

    inline void unsafe_set_gait_correlations(bolt::MatCRef<> new_G) {
        _.G = new_G;
        _.gait.unsafe_set_gait_correlations(new_G);
    }
};

} // namespace tgc

#endif // TGC_TWIST_GAIT_POSITIONAL_H_