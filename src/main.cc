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

constexpr double kControlRate = 1./100.; // Control loop frequency (Hz)
constexpr double kRenderRate = 1./60.;   // Render loop frequency (Hz)
constexpr double kRtf = 1.0;

// Convert a string to a 6D vector (linear and angular velocity)
bolt::Option<bolt::Vec<6>> VFromStr(bolt::str s) {
    bolt::Vec<6> V;

    if((std::stringstream((const char*)s.c_str()) >> V(0) >> V(1) >> V(2) >> V(3) >> V(4) >> V(5)).fail()) {
        return bolt::None;
    }

    return bolt::Some(V);
}

/*
void fix_base(Simulation& simulation) {
    auto& model = simulation.model();
    int root_joint_id = mj_name2id(&model, mjOBJ_JOINT, "root");
    //TODO
}
*/

int main(int argc, const char *argv[])
{
    // Parse command line arguments
    bolt::Config config = bolt::expect(bolt::Config::make(argc, argv), "Failed to parse args.");
    auto& args = config.args();

    bolt::expect(args.size() > 1, "MuJoCo XML file should be specified.");
    bolt::expect(args.size() > 2, "Gait specification YAML file should be specified.");

    double rtf = kRtf;
    if(config.has_key("rtf")) rtf = bolt::expect(config.get_value<double>("rtf"), "Missing or invalid -rtf value");

    // Initialize simulation, gait and viewer
    auto simulation = bolt::expect(Simulation::make_shared(args[1]), "Simulation should have loaded.");

    auto gait = tgc::TwistGait_Positional::make(YAML::LoadFile(args[2].c_str())).get();
    //std::cerr << gait.gait_correlations() << "\n";

    auto viewer = bolt::expect(Viewer::make_shared(simulation), "Viewer should be constructed.");

    /*
    tgc::Permutation P_sim2gait = tgc::QPermutation(
        simulation->joint_names(),
        gait.jointNames()
    ).get();
    */
    
    // Set command velocity
    bolt::Vec<6> V_cmd = bolt::Vec<6>{0,0,0,  0,0,0};
    if(config.has_key("Vcmd")) {
        V_cmd = bolt::expect(VFromStr(bolt::expect(config.get_value("Vcmd"), "Vcmd missing value")), "Invalid Vcmd");
    }
    
    double t_prev = 0.0; // Previous simulation time for reset detection

    // Run simulation loop
    while(!viewer->window_should_close()) {
        mjtNum render_start = simulation->sim_time();

        while(simulation->sim_time() - render_start < rtf * kRenderRate) {
            
            mjtNum control_start = simulation->sim_time();
            
            // Detect reset if current time is smaller than previous time
            if (simulation->sim_time() < t_prev) {
                gait.reset();
            }
            
            while(simulation->sim_time() - control_start < rtf * kControlRate) {
                simulation->step();
            }

            double t = simulation->mutable_data().time;
            t_prev = t; // Update previous time for the next iteration
            
            bolt::Vec<> q = simulation->joint_pos();

            // Randomize command velocity
            if (config.has_key("rand")) {
                bolt::Vec<6> V_rand = bolt::Vec<6>::Random();
                bolt::Linear(V_rand) *= 8 * 1e-1;
                bolt::Linear(V_rand).z() = 0.;
                bolt::Angular(V_rand).x() = 0.;
                bolt::Angular(V_rand).y() = 0.;
                double decay = 0.99;
                V_cmd = decay * V_cmd + (1. - decay) * V_rand;
            }

            /*
            if(t > 5.0) {
                V_cmd.setZero();
            }
            */

            // Measure gait controller step duration
            //auto gait_controller_start = std::chrono::high_resolution_clock::now();

            //--------------------------------------------------------------------------
            // Get joint torques
            bolt::Vec<> tau_ext = simulation->joint_ext_eff();
            // Compute next joint positions using gait controller
            bolt::Vec<> next_q = gait.next_q(V_cmd, q, tau_ext, t, rtf * kControlRate);
            //--------------------------------------------------------------------------

            /*
            auto gait_controller_duration = std::chrono::high_resolution_clock::now() - gait_controller_start;
            std::cerr << "gait controller duration: " << 1e-9 * gait_controller_duration.count() << '\n';
            */

            // Apply joint commands
            simulation->cmd_pos(next_q);
        }

        // Update viewer once enough time has been simulated
        viewer->update();
    }
}