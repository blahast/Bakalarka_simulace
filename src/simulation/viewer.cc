#include "simulation/viewer.h"

#include <iostream>
#include <chrono>

static std::weak_ptr<Viewer> singleton_viewer;
#define TRY_LOCK_self \
    std::shared_ptr<Viewer> locked_ptr = singleton_viewer.lock(); \
    if(!locked_ptr) return; \
    Viewer::Self& self = locked_ptr->self;

//
// GUI callbacks
//

void keyboard_callback(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    TRY_LOCK_self

    // backspace: reset simulation
    if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
        mj_resetData(
            &self.simulation->model(),
            &self.simulation->mutable_data());
        mj_forward(
            &self.simulation->model(),
            &self.simulation->mutable_data());
    }

    bool ctrl_mod = mods & GLFW_MOD_CONTROL;

    if(act==GLFW_PRESS && key==GLFW_KEY_Q && ctrl_mod) {
        glfwSetWindowShouldClose(self.window, GLFW_TRUE);
    }
}


void mouse_button_callback(GLFWwindow* window, int button, int act, int mods)
{
    TRY_LOCK_self

    // update mouse position
    glfwGetCursorPos(window, &self.gui_state.mouse_pos.x, &self.gui_state.mouse_pos.y);

    // update button state
    self.gui_state.mouse_left =
        glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS;
    self.gui_state.mouse_middle = 
        glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS;
    self.gui_state.mouse_right = 
        glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS;

    bool mod_shift = mods & GLFW_MOD_SHIFT;
    bool ctrl_mod = mods & GLFW_MOD_CONTROL;
    bool alt_mod = mods & GLFW_MOD_ALT;

    // body selection
    if(act==GLFW_PRESS && button==GLFW_MOUSE_BUTTON_LEFT && ctrl_mod) {
        // reset body selection
        if(mod_shift) {
            mjv_defaultPerturb(&self.perturb);
        } else {
            int width, height;
            glfwGetFramebufferSize(self.window, &width, &height);

            mjtNum selpnt[3];
            int selgeom, selflex, selskin;

            int body_id = mjv_select(
                &self.simulation->model(),
                &self.simulation->mutable_data(),
                &self.option,
                static_cast<mjtNum>(width)/ height,
                self.gui_state.mouse_pos.x / width,
                1 - self.gui_state.mouse_pos.y / height,
                &self.scene, selpnt, &selgeom, &selflex, &selskin);

            if(body_id >= 0) {
                // record selection
                self.perturb.select = body_id;
                self.perturb.flexselect = selflex;
                self.perturb.skinselect = selskin;

                // compute localpos
                mjtNum tmp[3];
                mju_sub3(tmp,
                    selpnt,
                    self.simulation->mutable_data().xpos + 3*self.perturb.select);
                mju_mulMatTVec(self.perturb.localpos,
                    self.simulation->mutable_data().xmat + 9*self.perturb.select,
                    tmp, 3, 3);
            }
        }
    }
    if(act==GLFW_PRESS && button==GLFW_MOUSE_BUTTON_RIGHT && ctrl_mod) {
        self.gui_state.perturb = true;
        self.perturb.active = mjPERT_TRANSLATE;
        mjv_initPerturb(
            &self.simulation->model(),
            &self.simulation->mutable_data(),
            &self.scene,
            &self.perturb);
    }
    if(act==GLFW_RELEASE && button==GLFW_MOUSE_BUTTON_RIGHT && self.gui_state.perturb) {
        self.perturb.active = 0;
        self.gui_state.perturb = false;

        mju_zero(
            self.simulation->mutable_data().xfrc_applied,
            6*self.simulation->model().nbody);
    }
}

void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos)
{
    TRY_LOCK_self

    // no buttons down: nothing to do
    if (!self.gui_state.mouse_left
        && !self.gui_state.mouse_middle
        && !self.gui_state.mouse_right)
    { return; }

    // get shift key state
    bool mod_shift =
        glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS
        || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS;
    bool mod_ctrl = 
        glfwGetKey(window, GLFW_KEY_LEFT_CONTROL)==GLFW_PRESS
        || glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL)==GLFW_PRESS;

    // compute mouse displacement, save
    double dx = xpos - self.gui_state.mouse_pos.x;
    double dy = ypos - self.gui_state.mouse_pos.y;
    self.gui_state.mouse_pos.x = xpos;
    self.gui_state.mouse_pos.y = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // determine action based on mouse button
    mjtMouse action;
    if (self.gui_state.mouse_right) {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (self.gui_state.mouse_left) {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
        action = mjMOUSE_ZOOM;
    }

    if (mod_ctrl || self.gui_state.perturb) {
        // move perturb
        mjv_movePerturb(
            &self.simulation->model(),
            &self.simulation->mutable_data(),
            action, dx/width, dy/height,
            &self.scene, &self.perturb);

    } else {
        // move camera
        mjv_moveCamera(
            &self.simulation->model(),
            action, dx/height, dy/height,
            &self.scene, &self.camera);
    }
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    TRY_LOCK_self

    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(
        &self.simulation->model(),
        mjMOUSE_ZOOM, 0, -0.05*yoffset,
        &self.scene, &self.camera);
}

/* static */
bolt::Option<std::shared_ptr<Viewer>> 
Viewer::
make_shared(std::shared_ptr<Simulation>& simulation)
{
    if(!singleton_viewer.expired()) return bolt::None;

    auto& model = simulation->model();
    auto& data = simulation->mutable_data();

    // init GLFW, create window, make OpenGL context current, request v-sync
    if(glfwInit() != GLFW_TRUE) return bolt::None;
    auto window = glfwCreateWindow(1200, 900, simulation->model().names, NULL, NULL);
    if(!window) return bolt::None;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // Prepare return value;
    auto viewer = std::shared_ptr<Viewer>(new Viewer({
        .simulation = simulation,
        .window = window
    }));
    singleton_viewer = viewer;
    auto& self = viewer->self;

    // initialize visualization data structures
    mjv_defaultCamera(&self.camera);
    mjv_defaultOption(&self.option);
    mjv_defaultScene(&self.scene);
    mjr_defaultContext(&self.context);
    mjv_defaultPerturb(&self.perturb);

    mjv_makeScene(&simulation->model(), &self.scene, 1000);
    mjr_makeContext(
        &simulation->model(), &self.context, mjFONTSCALE_100);

    // register callbacks
    glfwSetKeyCallback(self.window, keyboard_callback);
    glfwSetMouseButtonCallback(self.window, mouse_button_callback);
    glfwSetCursorPosCallback(self.window, cursor_pos_callback);
    glfwSetScrollCallback(self.window, scroll_callback);

    self.option.flags[mjVIS_PERTOBJ] = true;
    self.option.flags[mjVIS_PERTFORCE] = false;
    self.option.flags[mjVIS_SELECT] = true;

    return bolt::Some(singleton_viewer.lock());
}

Viewer::~Viewer()
{
    glfwDestroyWindow(self.window);
    glfwTerminate();
    glfwPollEvents();
    mjv_freeScene(&self.scene);
    mjr_freeContext(&self.context);
}

void
Viewer::
update()
{
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(
        self.window, &viewport.width, &viewport.height);

    mjv_updateScene(
        &self.simulation->model(),
        &self.simulation->mutable_data(),
        &self.option,
        &self.perturb,
        &self.camera,
        mjCAT_ALL,
        &self.scene);
    mjr_render(viewport, &self.scene, &self.context);

    glfwSwapBuffers(self.window);
    glfwPollEvents();

    mju_zero(
        self.simulation->mutable_data().xfrc_applied,
        6*self.simulation->model().nbody);
    // mjv_applyPerturbPose(
    //     &self.simulation->model(), 
    //     &self.simulation->mutable_data(),
    //     &self.perturb, 0);  // mocap bodies only
    mjv_applyPerturbForce(
        &self.simulation->model(), 
        &self.simulation->mutable_data(),
        &self.perturb);
}