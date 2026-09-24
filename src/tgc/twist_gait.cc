#include "tgc/twist_gait.h"

#include "bolt/base/iterator.h"
#include "bolt/kinematics/screw.h"

using namespace tgc;

std::tuple<double, double, double> Tau_TauMax_Tau0(
    bolt::VecCRef<6> V,
    bolt::VecCRef<3> p_tip,
    bolt::VecCRef<3> p0,
    double r_ws,
    double h_swing
) {
    bolt::Vec<3> d = p0 - p_tip;
    d.z() = 0;
    double tau, tau_max, tau0;
    if(V.isZero(0)) {
        tau = d.norm();
        tau_max = 1;
        tau0 = 1;
    } else if(bolt::Angular(V).isZero(0)) {
        bolt::Vec<3> vn = bolt::Linear(V).normalized();
        double v_norm = bolt::Linear(V).norm();

        tau = d.dot(vn)/v_norm;
        tau_max = sqrt(r_ws*r_ws - d.cross(vn).squaredNorm())/v_norm;
        tau0 = r_ws/v_norm;
    } else {
        bolt::Vec<3> v_c = bolt::Linear(V) - p0.cross(bolt::Angular(V));
        bolt::Vec<3> v_t = bolt::Linear(V) - p_tip.cross(bolt::Angular(V));
        double v_c_n = v_c.norm();
        double v_t_n = v_t.norm();
        double w = abs(bolt::Angular(V).z());

        tau = copysign(
            acos_abxx(
                v_c_n/v_t_n/2 + v_t_n/v_c_n/2,
                d.squaredNorm()/v_c_n/v_t_n/2,
                w
            )/w,
            v_c.dot(d)
        );
        tau_max = acos_abxx(
            v_c_n/v_t_n/2 + v_t_n/v_c_n/2,
            (r_ws/v_c_n)*(r_ws/v_t_n)/2,
            w
        )/w;
        tau0 = acos_abxx(1, pow(r_ws/v_c_n, 2)/2, w)/w;
    } 
    return {tau, tau_max, tau0};
}

/* static */
bolt::Result<TwistGait, TwistGait::eError>
TwistGait::
make(
    ShapingFunction a,
    ShapingFunction b,
    ShapingFunction c,
    bolt::MatCRef<> C,
    double tau_swing,
    double h_swing,
    double r_ws,
    const std::vector<bolt::SE3<>>& T0
) {
    double row_sum = C.row(0).sum();

    if(!C.rowwise().sum().cwiseEqual(row_sum).all()
        || C != C.transpose()
        || C.rows() != C.cols() 
        || C.rows() != T0.size()
        || !C.unaryExpr([](double c){ return -1. <= c && c <= 1.; }).all()
    ) return bolt::Err(eError::InvalidGaitCorrelations);

    double duty_factor = (T0.size() - row_sum)/(T0.size() + row_sum);

    std::vector<bolt::SE3<>> _T0(T0);
    std::vector<bolt::SE3<>> _T_tip(T0);
    std::vector<bolt::Option<double>> phi(T0.size());

    std::vector<bolt::Vec<6>> V_tip(T0.size(), bolt::Vec<6>::Zero());

    return bolt::Ok(TwistGait(
        a,
        b,
        c,
        C.matrix(),
        tau_swing,
        h_swing,
        r_ws,
        _T0,
        duty_factor,
        _T_tip,
        V_tip,
        phi
    ));
}

void
TwistGait::
reset()
{
    for(auto[tt, t0] : bolt::Zip(_self.T_tip, _self.T0)) {
        tt = t0;
    }
    for(auto& p : _self.phi) {
        p = bolt::None;
    }
    for(auto& v : _self.V_tip) {
        v.setZero();
    }
    // _self.is_V_updated = false;
}

void
TwistGait::
step(bolt::VecCRef<6> V, double delta)
{
    // Compute State -----------------------------------------------------------
    bolt::Vec<> tau(nLegs());
    bolt::Vec<> tau_max(nLegs());
    bolt::Vec<> tau0(nLegs());

    for(int i = 0; i < nLegs(); ++i) {
        std::tie(tau(i), tau_max(i), tau0(i)) =
            Tau_TauMax_Tau0(
                V,
                _self.T_tip[i].translation(),
                _self.T0[i].translation(),
                _self.r_ws,
                _self.h_swing
            );
    }

    // Initialize swing --------------------------------------------------------
    double theta_remain = kInfty;
    double theta_remain_candidate = kInfty;
    int swing_candidate = -1;
    for(int i = 0; i < nLegs(); ++i) {
        if(_self.phi[i].is_some()) continue;
        bool can_swing = true;
        for(int j = 0; j < nLegs(); ++j) {
            if(_self.phi[j].is_some()
                && _self.phi[j].get() <= (1. - _self.C(i,j))/2.
            ) {
                can_swing= false;
            }
        }
        theta_remain = std::min(theta_remain, tau_max(i) - tau(i));
        can_swing = can_swing && (!V.isZero(0) || !tau.isZero(5e-2)); //TODO: Make into param.
        if(can_swing && tau_max(i) - tau(i) < theta_remain_candidate) {
            theta_remain_candidate = tau_max(i) - tau(i);
            swing_candidate = i;
        }
    }
    if(swing_candidate >= 0) {
        for(int i = 0; i < nLegs(); ++i) {
            if(_self.C(i, swing_candidate) == 1) _self.phi[i] = bolt::Some(0.);
        }
    }

    // Compute tip twists ------------------------------------------------------
    double scaled_tau_swing = fmin(
        _self.tau_swing,
        2 * tau0.minCoeff() / _self.duty_factor
    );
    bolt::Vec<6> V_swing = _self.duty_factor * V;
    bolt::Vec<6> V_lift = {0,0,0, 0,0,_self.h_swing/scaled_tau_swing};
    for(int i = 0; i < nLegs(); ++i) {
        if(_self.phi[i].is_some()) {
            double& phi = _self.phi[i].get();


            bolt::SE3<> T_target =
                bolt::se3_exp(_self.duty_factor * scaled_tau_swing/2 * V) * _self.T0[i];
            bolt::SE3<> T_touchdown =
                bolt::se3_exp(scaled_tau_swing*(
                    (1. - phi) * V_swing
                    - _self.a.I(phi, 1.) * (V + V_swing)
                    + _self.b.I(phi, 1.) * V_lift
                )) * _self.T_tip[i];
            bolt::Vec<6> V_err = bolt::SE3_Ad(_self.T_tip[i],
                bolt::SE3_log(T_touchdown.inverse() * T_target)
            );

            _self.V_tip[i] = 
                V_swing
                - _self.a(phi) * (V + V_swing)
                + _self.b(phi) * V_lift
                // + log(0.1)/scaled_tau_swing  * _self.c(phi) * V_err;  //TODO: Make into param.
                + 2. * _self.c(phi) * V_err;  //TODO: Make into param.
        } else if (theta_remain > 0.) {
            _self.V_tip[i] = -V;
        } else {
            _self.V_tip[i].setZero();
        }
    }
    // integrate state ---------------------------------------------------------
    for(int i = 0; i < nLegs(); ++i) {
        if(_self.phi[i].is_some()) {
            _self.phi[i].get() += delta / scaled_tau_swing;
            if(_self.phi[i].get() >= 1.) _self.phi[i] = bolt::None;
        }

        _self.T_tip[i] = bolt::se3_exp(delta * _self.V_tip[i]) * _self.T_tip[i];
    }
}
