#pragma once

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "bolt/base/option.h"

#include "simulation/simulation.h"
#include "simulation/memory.h"

class Viewer {
    struct Self {
        std::shared_ptr<Simulation> simulation;

        GLFWwindow* window;
        mjvCamera camera;
        mjvOption option;
        mjvScene scene;
        mjrContext context;

        mjvPerturb perturb;

        struct {
            bool mouse_left = false;
            bool mouse_middle = false;
            bool mouse_right =  false;

            bool perturb = false;

            struct {
                double x = 0.;
                double y = 0.;
            } mouse_pos;
        } gui_state;

    } self;

    explicit inline Viewer(Self&& self) : self(move(self)) {}

    // GUI callback
    friend void keyboard_callback(GLFWwindow* window, int key, int scancode, int act, int mods);
    friend void mouse_button_callback(GLFWwindow* window, int button, int act, int mods);
    friend void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos);
    friend void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
public:
    virtual ~Viewer();
    static bolt::Option<std::shared_ptr<Viewer>>
        make_shared(std::shared_ptr<Simulation>& simulation);


    inline bool window_should_close() const { return glfwWindowShouldClose(self.window); }

    void update();

    inline Simulation& mutable_simulation() { return *self.simulation; }

};