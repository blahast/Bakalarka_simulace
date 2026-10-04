#include "simulation/simulation.h"

#include <iostream>

bolt::Option<std::shared_ptr<Simulation>>
Simulation::
make_shared(const bolt::str xml_path)
{
    char error[1000] = "Unknown MuJoCo error";
    
    auto model = unique_c_ptr<mjModel>(
        mj_loadXML(xml_path.c_str(), NULL, error, 1000), mj_deleteModel);
        
    if(!model) {
        std::cerr << "MuJoCo Load Error: " << error << std::endl;
        return bolt::None;
    }

    auto data = unique_c_ptr<mjData>(
        mj_makeData(model.get()), mj_deleteData);
    if(!data) return bolt::None;

    for(int i = 0; i < model->nu; ++i) {
        int jnt_id = model->actuator_trnid[2 * i];
        data->ctrl[i] = data->qpos[model->jnt_qposadr[jnt_id]];
    }

    mj_forward(model.get(), data.get());

    return bolt::Some(std::shared_ptr<Simulation>(new Simulation(Self{
        .mj_model = move(model),
        .mj_data = move(data)
    })));
}

void
Simulation::
step()
{
    mj_step(self.mj_model.get(), self.mj_data.get());
}

std::vector<bolt::str>
Simulation::
joint_names() const
{
    auto& model = *self.mj_model;
    std::vector<bolt::str> names_ret;
    names_ret.reserve(model.nu);

    for(int i = 0; i < model.nu; ++i) {
        names_ret.emplace_back(bolt::str::make_unchecked(
            mj_id2name(&model, mjOBJ_JOINT, model.actuator_trnid[2*i])));
    }

    return names_ret;
}

bolt::Vec<> 
Simulation::
joint_pos() const
{
    auto& model = *self.mj_model;
    auto& data = *self.mj_data;
    bolt::Vec<> pos(model.nu);

    for(int i = 0; i < model.nu; ++i) {
       pos(i) = data.qpos[model.jnt_qposadr[model.actuator_trnid[2*i]]];
    }

    return pos;
}

bolt::Vec<> 
Simulation::
joint_vel() const
{
    auto& model = *self.mj_model;
    auto& data = *self.mj_data;
    bolt::Vec<> vel(model.nu);

    for(int i = 0; i < model.nu; ++i) {
       // Use jnt_dofadr for velocity indexing
       vel(i) = data.qvel[model.jnt_dofadr[model.actuator_trnid[2*i]]];
    }

    return vel;
}

bolt::Vec<> 
Simulation::
joint_acc() const
{
    auto& model = *self.mj_model;
    auto& data = *self.mj_data;
    bolt::Vec<> acc(model.nu);

    for(int i = 0; i < model.nu; ++i) {
       // Use jnt_dofadr for acceleration indexing
       acc(i) = data.qacc[model.jnt_dofadr[model.actuator_trnid[2*i]]];
    }

    return acc;
}

bolt::Vec<>
Simulation::
joint_eff() const
{
    auto& model = *self.mj_model;
    auto& data = *self.mj_data;
    bolt::Vec<> effort(model.nu);

    for(int i = 0; i < model.nu; ++i) {
       effort(i) = data.actuator_force[i];
    }

    return effort;
}

// Returns torques from external forces (and joint frictionloss) (N*m)
bolt::Vec<> Simulation::joint_ext_eff() const
{
    auto& model = *self.mj_model;
    auto& data = *self.mj_data;
    bolt::Vec<> ext_effort(model.nu);

    for(int i = 0; i < model.nu; ++i) {
        // Get the degree of freedom address for the current actuator
        int dof_adr = model.jnt_dofadr[model.actuator_trnid[2*i]];
        
        // Read the constraint force directly from MuJoCo data
        ext_effort(i) = data.qfrc_constraint[dof_adr];
    }

    return ext_effort;
}

void
Simulation::
cmd_pos(Eigen::Ref<const bolt::Vec<>> q) const
{
    if(q.size() != self.mj_model->nu) bolt::panic("Control dimensions mismatch.");

    for(int i = 0; i < q.size(); ++i) {
        self.mj_data->ctrl[i] = q(i);
    }
}