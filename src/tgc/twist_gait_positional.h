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
        bolt::KinematicChain          leg_model;        // Leg kinematic chain model
        std::vector<bolt::SE3Store<>> leg_Tb;           // Leg base transforms (from body to leg base)
        std::vector<std::string>      leg_names;        // Leg names
        bolt::VecStore<>              nominal_leg_q;    // Nominal leg joint positions
        std::vector<std::string>      leg_joint_names;  // Leg joint names
        double                        ws_r;             // Workspace radius

        double                        step_h;           // Step height
        double                        t_stance;         // Stance time
        double                        t_swing;          // Swing time
        bolt::VecStore<>              leg_gait_offset;  // Leg gait offsets (phase offsets for each leg)
        bolt::MatStore<>              G;                // Gait correlations

        // Admittance parameters
        bool                          adm_enable;       // Enable flag for admittance control
        bool                          adm_robot_weight_compensation; // Enable flag for robot weight compensation
        double                        adm_force_touchdown_threshold; // Force threshold for touchdown detection
        double                        adm_robot_mass;   // Robot mass for weight compensation
        double                        adm_stiffness;    // Admittance stiffness (K)
        double                        adm_damping;      // Admittance damping (B), calculated from ratio
        double                        adm_mass;         // Admittance mass (M)
        double                        expected_force_multiplier; // Multiplier for expected force in admittance control
        double                        adm_ramp_up_time; // Ramp up time for expected force

        TwistGait gait;                                 // Twist gait controller (generates leg tip positions)

        // State --------------------------------------------------------------
        bolt::VecStore<>              q;                // Current joint angles
        std::vector<double>           adm_pos_z;        // Admittance position state in Z axis
        std::vector<double>           adm_vel_z;        // Admittance velocity state in Z axis
        std::vector<bolt::Vec<3>>     tip_forces;       // Tip forces in base frame for all legs
        std::vector<double>           expected_fz;      // Expected Z forces for all legs
        std::vector<double>           force_ramp_time;  // Timer for expected force ramp-up
    } _;

    template<typename... Args>
    explicit TwistGait_Positional(Args&&... args) : _{std::move<Args>(args)...} {
        reset();
    }
public:
    // Creates a TwistGait_Positional instance from YAML configuration data
    static bolt::Option<TwistGait_Positional> make(const YAML::Node& yml_data);

    void printLegs() const;                         // Prints leg and joint names
    std::vector<std::string> jointNames() const;    // Returns leg and joint names

    inline int nLegs() const { return _.leg_Tb.size(); }                // Returns number of legs
    inline int nDof() const { return nLegs() * _.leg_model.nDof(); }    // Returns number of degrees of freedom (joints)

    void reset();   // Resets the gait controller and joint angles to their initial state

    bolt::Vec<> nominal_q() const; // Returns the nominal joint angles for all legs in a single vector

    // Returns the tip forces in base frame for all legs
    inline const std::vector<bolt::Vec<3>>& tipForces() const {
        return _.tip_forces;
    }

    // Returns the expected tip forces in base frame for all legs
    inline const std::vector<double>& expTipForces() const {
        return _.expected_fz;
    }

    // Returns the next joint angles for all legs
    bolt::Vec<> next_q(
        bolt::VecCRef<6> V,
        bolt::VecCRef<>  q,
        bolt::VecCRef<>  tau,  // New: Added tau for admittance control
        double           t,
        double           dt);

    // Returns current gait correlations matrix
    inline const bolt::MatStore<>& gait_correlations() const {
        return _.G;
    }

    // Sets new gait correlations matrix
    inline void unsafe_set_gait_correlations(bolt::MatCRef<> new_G) {
        _.G = new_G;
        _.gait.unsafe_set_gait_correlations(new_G);
    }

private:
    // Applies admittance control to the target transform based on joint torques
    bolt::SE3<> apply_admittance(const bolt::SE3<>& T_target, double measured_fz, double expected_fz, double dt, int i);
    
    // Computes the measured force at the tip of leg
    bolt::Vec<3> compute_measured_force(int i, bolt::VecCRef<> tau_ext_leg, const bolt::SE3<>& Tb);
};

} // namespace tgc

#endif // TGC_TWIST_GAIT_POSITIONAL_H_