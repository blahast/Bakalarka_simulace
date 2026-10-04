#include "tgc/twist_gait_positional.h"
#include "tgc/parser.h"
#include "tgc/math.h"

#include "tgc/twist_gait.h"
#include "tgc/shaping_functions.h"

#include "bolt/kinematics/screw.h"
#include "bolt/base/option.h"
#include "bolt/eigen/dense.h"
#include "bolt/base/expect.h"
#include "bolt/base/panic.h"
#include "bolt/base/iterator.h"

#include <cmath>
#include <iostream>
#include <tuple>

namespace tgc {

double a(double t, double T) { return cos(2*M_PI*t/T); }
double A(double t, double T) { return T/2/M_PI*sin(2*M_PI*t/T); }
double int_a(double t1, double t2, double T) { return A(t2, T) - A(t1, T); }

double b(double t, double T) { return M_PI/T*sin(2*M_PI*t/T); }
double B(double t, double T) { return -.5*cos(2*M_PI*t/T); }
double int_b(double t1, double t2, double T) { return B(t2, T) - B(t1, T); }

double c(double t, double T) { return 1 - cos(2*M_PI*t/T); }
// double C(double t, double T) { return t - T/2/M_PI*cos(2*M_PI*t/T); }
// double int_c(double t1, double t2, double T) { return C(t2, T) - C(t1, T); }

inline double BoundedReciprocal(double x, double ub) {
    bolt::expect(x != 0, "Value s/b non-zero.");
    bolt::expect(ub > 0, "Upper bound s/b positive.");
    return std::max(std::min(1/x, ub), -ub);
    // return tanh(ub*x)/abs(x);
}

bolt::Mat<> GaitCorrelations(
    double t_stance,
    double t_swing,
    bolt::VecCRef<> leg_gait_offset);

Eigen::Map<const bolt::Vec<>> EigenFromStd(const std::vector<double>& v);

bolt::Option<double> GammaCosLaw(double a, double b, double c);

std::tuple<double, double, double> Theta_ThetaBound_ThetaBoundMax(
    bolt::VecCRef<6> V,
    bolt::VecCRef<3> tip_position,
    bolt::VecCRef<3> ws_center,
    double ws_r,
    double step_h);

bolt::Option<bolt::SE3<>> TranformFromXYZRPY(const YAML::Node& array6);

bolt::Option<bolt::Vec<>> VectorFromYAML(const YAML::Node& array);

bolt::Option<bolt::KinematicChain> KinematicChainFromYML(const YAML::Node& yml);

template<typename T>
T getOr(bolt::Option<T> opt, T alt) {
    if(opt.is_none()) return alt;
    return opt.get();
}

/* static */
bolt::Option<TwistGait_Positional> TwistGait_Positional::make(const YAML::Node& yml_data)
{
    try {
        if(!yml_data.IsDefined()) return bolt::None;

        auto leg_tmpl_yml = yml_data["leg_tmpl"];
        if(!leg_tmpl_yml.IsDefined()) return bolt::None;
        if(
            leg_tmpl_yml["names"].size() + 1 != leg_tmpl_yml["transforms"].size()
            || leg_tmpl_yml["names"].size() != leg_tmpl_yml["nominal"].size())
        {
            return bolt::None;
        }

        // Leg template
        auto leg_model = KinematicChainFromYML(leg_tmpl_yml["transforms"]).get();

        // Leg ws
        if(!leg_tmpl_yml["ws_radius"].IsDefined()) return bolt::None;
        double wsr = leg_tmpl_yml["ws_radius"].as<double>();

        // Nominal angles (rad)
        bolt::Vec<> nominal_leg_q = M_PI/180 * VectorFromYAML(leg_tmpl_yml["nominal"]).get();

        double t_stance = yml_data["gait"]["stance"].as<double>();
        double t_swing = yml_data["gait"]["swing"].as<double>();
        double step_h = yml_data["gait"]["step_height"].as<double>();

        // Legs
        std::vector<bolt::Unaligned_t<bolt::SE3<>>> leg_Tb;
        std::vector<std::string> leg_names;
        std::vector<double> leg_gait_offset;
        auto legs_yml = yml_data["legs"];

        if(!legs_yml.IsSequence()) return bolt::None;

        // Parse leg base transforms, names, and gait offsets
        for(auto leg : legs_yml) {
            bolt::SE3<> leg_base_T = TranformFromXYZRPY(leg["base"]).get();
            leg_Tb.emplace_back(leg_base_T);
            leg_names.emplace_back(std::string(leg["name"].as<std::string>()));
            leg_gait_offset.push_back(leg["offset"].as<double>());
        }

        // Joint names
        std::vector<std::string> leg_joint_names;
        for(auto jn : leg_tmpl_yml["names"]) {
            leg_joint_names.push_back(jn.as<std::string>());
        }

        bool adm_enable = false;
        double adm_stiffness = 0.0;
        double adm_damping = 0.0;
        double adm_mass = 0.0;

        auto admittance_yml = yml_data["admittance"];
        if(admittance_yml.IsDefined()) {
            // Check if admittance is enabled
            adm_enable = admittance_yml["enable"].IsDefined() ? admittance_yml["enable"].as<bool>() : false;
            
            if(adm_enable) {
                if(!admittance_yml["stiffness"].IsDefined() || !admittance_yml["damping_ratio"].IsDefined() || !admittance_yml["mass"].IsDefined()) {
                    return bolt::None;
                }
                adm_stiffness = admittance_yml["stiffness"].as<double>();
                double damping_ratio = admittance_yml["damping_ratio"].as<double>();
                adm_mass = admittance_yml["mass"].as<double>();
                
                // Calculate absolute damping coefficient B
                adm_damping = damping_ratio * 2.0 * std::sqrt(adm_stiffness * adm_mass);
            }
        }

        // Gait correlations
        bolt::Mat<> C = GaitCorrelations(t_stance, t_swing, EigenFromStd(leg_gait_offset));

        // Compute nominal leg tip positions (robot base frame)
        std::vector<bolt::SE3<>> T0;
        for(int i = 0; i < leg_Tb.size(); ++i) {
            T0.emplace_back(
                leg_Tb[i] * leg_model.FK(nominal_leg_q)
            );
        }

        // Create gait controller
        TwistGait gait = bolt::expect(
            TwistGait::make(
                StrideCos(),
                LiftSin(),

                // GainSin(),
                GainCos(),

                C,
                1.,
                step_h,
                wsr,
                T0
            ), 
            "Failed to init gait."
        );

        return bolt::Some(TwistGait_Positional(
            leg_model,
            leg_Tb,
            leg_names,
            nominal_leg_q,
            leg_joint_names,
            wsr,
            step_h,
            t_stance,
            t_swing,
            EigenFromStd(leg_gait_offset).matrix(),
            C,
            adm_enable,
            adm_stiffness,
            adm_damping,
            adm_mass,
            gait
        ));
    } catch(YAML::Exception e) {
        std::cerr << e.what() << std::endl;
        return bolt::None;
    }
}

void TwistGait_Positional::reset()
{
    _.gait.reset();
    _.q = nominal_q();

    // Initialize admittance states to zero
    _.adm_pos_z.assign(nLegs(), 0.0);
    _.adm_vel_z.assign(nLegs(), 0.0);
}

void TwistGait_Positional::printLegs() const
{
    for(auto ln : _.leg_names) {
        std::cout << ln << std::endl;
        for(auto jn : _.leg_joint_names) {
            std::cout << '\t' << jn << "_" << ln << std::endl;
        }
    }
}

std::vector<std::string> TwistGait_Positional::jointNames() const
{
    std::vector<std::string> names;
    for(auto ln : _.leg_names) {
        for(auto jn : _.leg_joint_names) {
            names.push_back(jn + "_" + ln);
        }
    }
    return names;
}

bolt::Vec<> TwistGait_Positional::nominal_q() const
{
    bolt::Vec<> q(nDof());
    for(int i = 0; i < q.size(); i+=_.leg_model.nDof()) {
        q.segment(i, _.leg_model.nDof()) = _.nominal_leg_q;
    }
    return q;
}

bolt::Vec<> TwistGait_Positional::next_q(
    bolt::VecCRef<6> V,
    bolt::VecCRef<>  q,
    bolt::VecCRef<>  tau,
    double           t,
    double           dt)
{
    bolt::expect(bolt::Linear(V).z() == 0. && bolt::Angular(V).x() == 0. && bolt::Angular(V).y() == 0., "Command velocity shoul be planar.");

    // Create references to each leg's joint angles
    std::vector<bolt::MatRef<>> ql;
    for(int i = 0; i < nLegs(); ++i) {
        ql.emplace_back(_.q.segment(i * _.leg_model.nDof(), _.leg_model.nDof()));
    }

    int i = 0;
    for(auto[Tt, qli, Tb] : bolt::Zip(_.gait.T_tip(), ql, _.leg_Tb)) {
        auto tau_i = tau.segment(i * _.leg_model.nDof(), _.leg_model.nDof());
        
        // Apply admittance control to adjust the target tip transform based on external torques
        bolt::SE3<> T_adj = Tt;
        if (_.adm_enable) {
            T_adj = apply_admittance(Tt, tau_i, Tb, dt, i);
        }

        qli = bolt::expect(_.leg_model.IK((Tb.inverse() * T_adj).translation(), qli), "IK failure.");
        i++;
    }

    _.gait.step(V, dt);

    return _.q;
}

bolt::SE3<> TwistGait_Positional::apply_admittance(
    const bolt::SE3<>& T_target,    // Target tip transform in robot base frame
    bolt::VecCRef<> tau_ext_leg,    // External torques for the leg's joints
    const bolt::SE3<>& Tb,          // Leg base transform (from robot base to leg base)
    double dt,                      // Time step for the control loop
    int i                           // Index of the leg being processed
)                    
{
    // Apply deadband to compensate for friction loss
    bolt::Vec<> tau_active = tau_ext_leg.unaryExpr([](double tau) {
        const double friction_loss = 0.03;
        if (tau > friction_loss) return tau - friction_loss;
        if (tau < -friction_loss) return tau + friction_loss;
        return 0.0;
    });

    // Get the current joint angles for the leg
    int dof_per_leg = _.leg_model.nDof();
    bolt::Vec<> q_leg = _.q.segment(i * dof_per_leg, dof_per_leg);

    // Compute forward kinematics and base Jacobian in the leg's local coordinate frame
    bolt::SE3<double> T = _.leg_model.FK(q_leg);
    bolt::Mat<6, -1, double> jac = _.leg_model.J(q_leg);

    // Extract only the linear part of the Jacobian and shift to the end-effector
    bolt::Mat<3, -1, double> jac_linear = jac.bottomRows<3>() + jac.topRows<3>().colwise().cross(T.translation());

    // Calculate tip force from torques using robust solver to prevent instability near singularities
    bolt::Vec<3> tip_force_local = jac_linear.transpose().completeOrthogonalDecomposition().solve(tau_active);

    // Transform the computed local force into the robot's base coordinate frame
    bolt::Vec<3> tip_force_base = Tb.rotation() * tip_force_local;

    // Print tip force for debugging
    /*
    if (i == 0) {
        std::cout << "Leg " << i << " tip force (base frame): "<< tip_force_base.transpose() << std::endl;
    }
    */

    // Calculate Z-axis acceleration: a = (F - B*v - K*x) / M
    double f_z = tip_force_base.z();
    double a_z = (f_z - _.adm_damping * _.adm_vel_z[i] - _.adm_stiffness * _.adm_pos_z[i]) / _.adm_mass;

    // Integrate acceleration to get velocity and position (Semi-implicit Euler)
    _.adm_vel_z[i] += a_z * dt;
    _.adm_pos_z[i] += _.adm_vel_z[i] * dt;

    // Apply the calculated admittance displacement to the target transform
    bolt::SE3<> T_adj = T_target;
    T_adj.translation().z() += _.adm_pos_z[i];

    return T_adj;
}

// Helper functions -----------------------------------------------------------

bolt::Mat<> GaitCorrelations(double t_stance, double t_swing, bolt::VecCRef<> leg_gait_offset)
{
    int n = leg_gait_offset.size();

    bolt::Mat<> C(n, n);
    for(int i = 0; i < n; ++i) {
        for(int j = 0; j <= i; ++j) {
            double offset = std::abs(leg_gait_offset[i] - leg_gait_offset[j]);
            if(offset >= t_swing) offset = std::abs(offset - t_stance - t_swing);
            C(i,j) = C(j,i) = offset < t_swing ? 1 - 2*offset/t_swing : -1;
        }
    }

    return C;
}

Eigen::Map<const bolt::Vec<>> EigenFromStd(const std::vector<double>& v)
{
    return Eigen::Map<const bolt::Vec<>>(v.data(), v.size());
}

bolt::Option<double> GammaCosLaw(double a, double b, double c)
{
    double cg = (a*a + b*b - c*c)/2/a/b;
    if (std::abs(cg) > 1) return bolt::None;
    return bolt::Some(std::acos(cg));
}

std::tuple<double, double, double> Theta_ThetaBound_ThetaBoundMax(
    bolt::VecCRef<6> V,
    bolt::VecCRef<3> tip_position,
    bolt::VecCRef<3> ws_center,
    double ws_r,
    double step_h)
{
    bolt::Vec<3> d = ws_center - tip_position;
    d.z() = 0;
    double theta, theta_bound, theta_bound_max;
    if(V.isZero(0)) {
        theta = d.norm();
        theta_bound = 1;
        theta_bound_max = 1;
    } else if(bolt::Angular(V).isZero(0)) {
        bolt::Vec<3> vn = bolt::Linear(V).normalized();
        double v_norm = bolt::Linear(V).norm();

        theta = d.dot(vn)/v_norm;
        theta_bound = sqrt(ws_r*ws_r - d.cross(vn).squaredNorm())/v_norm;
        theta_bound_max = ws_r/v_norm;
    } else {
        bolt::Vec<3> v_c = bolt::Linear(V) - ws_center.cross(bolt::Angular(V));
        bolt::Vec<3> v_t = bolt::Linear(V) - tip_position.cross(bolt::Angular(V));
        double v_c_n = v_c.norm();
        double v_t_n = v_t.norm();
        double w = abs(bolt::Angular(V).z());

        theta = copysign(
            acos_abxx(
                v_c_n/v_t_n/2 + v_t_n/v_c_n/2,
                d.squaredNorm()/v_c_n/v_t_n/2,
                w
            )/w,
            v_c.dot(d)
        );
        theta_bound = acos_abxx(
            v_c_n/v_t_n/2 + v_t_n/v_c_n/2,
            (ws_r/v_c_n)*(ws_r/v_t_n)/2,
            w
        )/w;
        theta_bound_max = acos_abxx(1, pow(ws_r/v_c_n, 2)/2, w)/w;
    } 
    return {theta, theta_bound, theta_bound_max};
}

bolt::Option<bolt::SE3<>> TranformFromXYZRPY(const YAML::Node& array6)
{
    if(!array6.IsSequence() || array6.size() != 6) return bolt::None;

    try{
        return bolt::Some(bolt::SE3<>(
            Eigen::Translation3d(bolt::Vec<3>{
                array6[0].as<double>(),
                array6[1].as<double>(),
                array6[2].as<double>()
            })
            * Eigen::AngleAxisd(
                array6[5].as<double>() * M_PI/180, bolt::Vec<3>::UnitZ())
            * Eigen::AngleAxisd(
                array6[4].as<double>() * M_PI/180, bolt::Vec<3>::UnitY())
            * Eigen::AngleAxisd(
                array6[3].as<double>() * M_PI/180, bolt::Vec<3>::UnitX())
        )); 
    } catch(const YAML::BadConversion&) {
        return bolt::None;
    }
}

bolt::Option<bolt::Vec<>> VectorFromYAML(const YAML::Node& array)
{
    if(!array.IsSequence()) return bolt::None;

    try {
        bolt::Vec<> ret(array.size());
        for(int i = 0; i < array.size(); ++i) {
            ret(i) = array[i].as<double>();
        }
        return bolt::Some(ret);
    } catch(const YAML::BadConversion&) {
        return bolt::None;
    }
}

bolt::Option<bolt::KinematicChain> KinematicChainFromYML(const YAML::Node& yml)
{
    if(!yml.IsSequence()) return bolt::None;

    std::vector<bolt::Unaligned_t<bolt::Vec<6>>> screw_axes;
    bolt::SE3<> T = bolt::SE3<>::Identity();
    for(auto xyzrpy : yml) {
        T = T * TranformFromXYZRPY(xyzrpy).get();
        screw_axes.push_back(bolt::SE3_Ad(T, bolt::Vec<6>{0,0,1,0,0,0}));
    }
    screw_axes.pop_back();

    // return tgc::KinematicChain::make(screw_axes, T);
    auto ret = bolt::KinematicChain::make(screw_axes, T);
    if(ret.is_err()) return bolt::None;
    return bolt::Some(ret.get());
}


} // namespace tgc