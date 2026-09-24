#include <iostream>
#include <iomanip>
#include <utility>

#include "bolt/base/option.h"
#include "bolt/base/panic.h"
#include "bolt/base/expect.h"
#include "bolt/parsers/config.h"

#include "simulation/simulation.h"
#include "simulation/viewer.h"

#include "tgc/twist_gait_positional.h"
#include "tgc/index.h"

constexpr double kRate = 1./60.;
constexpr double kRtf = 1.0;

bolt::Option<bolt::Vec<6>> VFromStr(bolt::str s) {
    bolt::Vec<6> V;

    if((std::stringstream((const char*)s.c_str())
        >> V(0) >> V(1) >> V(2) >> V(3) >> V(4) >> V(5)
    ).fail()) return bolt::None;
     
    return bolt::Some(V);
}

void fix_base(Simulation& simulation) {
    auto& model = simulation.model();
    int root_joint_id = mj_name2id(&model, mjOBJ_JOINT, "root");
    //TODO
}

int main(int argc, const char *argv[])
{
    bolt::Config config = bolt::Config::make(argc, argv)
        .expect("Failed to parse args.");
    auto& args = config.args();

    bolt::expect(args.size() > 1, "MuJoCo XML file should be specified.");
    bolt::expect(args.size() > 2, "Gait specification YAML file should be specified.");

    double rtf = kRtf;
    if(config.has_key("rtf")) rtf = config.get_value<double>("rtf")
        .expect("Missing or invalid -rtf value");

    auto simulation = Simulation::make_shared(args[1])
        .expect("Simulation should have loaded.");

    auto gait = tgc::TwistGait_Positional::make(YAML::LoadFile(args[2].c_str())).peel();
    std::cerr << gait.gait_correlations() << "\n";

    auto viewer = Viewer::make_shared(simulation)
        .expect("Viewer should be constructed.");

    tgc::Permutation P_sim2gait = tgc::QPermutation(
        simulation->joint_names(),
        gait.jointNames()
    ).peel();
    
    bolt::Vec<6> V_cmd = bolt::Vec<6>{0,0,0,  1e-1,0,0};
    if(config.has_key("Vcmd")) {
        V_cmd = VFromStr(
            config.get_value("Vcmd").expect("Vcmd missing value")
        ).expect("Invalid Vcmd");
    }

    while(!viewer->window_should_close()) {
        mjtNum simstart = simulation->sim_time();
        while(simulation->sim_time() - simstart < rtf * kRate) {
            simulation->step();
        }
        viewer->update();

        // gait control
        double t = simulation->mutable_data().time;
        if(t == 0) gait.reset();
        bolt::Vec<> q = simulation->joint_pos();

        if (config.has_key("rand")) {
            bolt::Vec<6> V_rand = bolt::Vec<6>::Random();
            bolt::Linear(V_rand) *= 8 * 1e-1;
            bolt::Linear(V_rand).z() = 0.;
            // bolt::Angular(V_rand) *= 0.5;
            // bolt::Angular(V_rand) *= 0;
            bolt::Angular(V_rand).x() = 0.;
            bolt::Angular(V_rand).y() = 0.;
            double decay = 0.99;
            V_cmd = decay * V_cmd + (1. - decay) * V_rand;
        }
        // bolt::Angular(V_cmd) = bolt::Vec<3>{0,0,0.5};
        // bolt::Linear(V_cmd) = 0.3 * bolt::Vec<3>{sin(t/2),cos(t/2),0};
        // V_cmd *= 0.4;

        // if(t > 1.0) {
        //     V_cmd.setZero();
        // } else {
        //     V_cmd = bolt::Vec<6>{0,0,0,10e-2,0,0};
        // }

        auto gait_controller_start = std::chrono::high_resolution_clock::now();
        bolt::Vec<> next_q = gait.next_q(V_cmd, q, t, rtf * kRate);
        auto gait_controller_duration =
            std::chrono::high_resolution_clock::now() - gait_controller_start;
        // std::cerr
        //     << "gait controller duration: "
        //     << 1e-9 * gait_controller_duration.count() << '\n';

        simulation->cmd_pos(next_q);
    }
}