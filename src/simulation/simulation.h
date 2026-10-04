#pragma once

#include <vector>

#include "bolt/base/option.h"
#include "bolt/base/str.h"
#include "bolt/eigen/dense.h"

#include <mujoco/mujoco.h>

#include "simulation/memory.h"

class Simulation {
    struct Self {
        unique_c_ptr<mjModel> mj_model;
        unique_c_ptr<mjData> mj_data;
    } self;

    explicit inline Simulation(Self&& self) : self(move(self)) {}
public:
    static bolt::Option<std::shared_ptr<Simulation>> make_shared(const bolt::str xml_path);

    inline const mjModel& model() const { return *self.mj_model; }
    inline mjData& mutable_data() const { return *self.mj_data; }

    void step();
    inline mjtNum sim_time() const { return self.mj_data->time; }

    inline int n_cdof() const { return self.mj_model->nu; }

    std::vector<bolt::str> joint_names() const;

    bolt::Vec<> joint_pos() const;
    bolt::Vec<> joint_vel() const;
    bolt::Vec<> joint_acc() const;
    bolt::Vec<> joint_eff() const;
    bolt::Vec<> joint_ext_eff() const;

    void cmd_pos(Eigen::Ref<const bolt::Vec<>> q) const;

};

