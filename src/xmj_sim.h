#ifndef XMJ_SIM_ENV_H
#define XMJ_SIM_ENV_H

#include "stdio.h"
#include "string.h"

#include <thread>
#include <mutex>
#include <chrono>
#include <memory>
#include <mutex>
#include <condition_variable>

#include "simulator.h"
// #include <ros/ros.h>

namespace mj = ::mujoco;

class XBotMjSim {
public:

    typedef std::unique_ptr<XBotMjSim> UniquePtr;
    
    using clock = std::chrono::high_resolution_clock;

    XBotMjSim(
        const std::string model_fname,
        const std::string xbot2_config_path = "",
        bool headless = false,
        bool manual_stepping = false,
        int init_steps = 1,
        int timeout = 10,
        const std::string base_link_name = "base_link",
        bool match_rt_factor = false,
        double rt_factor_trgt = 1.0,
        bool render_to_file = false,
        std::string custom_camera_name = "custom_camera",
        std::string render_base_path = "/tmp",
        float render_fps = 60.0);
    ~XBotMjSim();

    bool is_running();
    bool step();
    bool reset();
    void close();
    
    int n_jnts();
    std::vector<std::string> jnt_names();

    void move_to_homing_now();
    void move_base_to_now(std::vector<double> p, std::vector<double> q);
    
    void read_state();

    std::vector<double> p_i = {0.0,0.0,1.0};
    std::vector<double> q_i = {1.0,0.0,0.0,0.0};
    std::vector<double> p = {0.0,0.0,1.0};
    std::vector<double> q = {1.0,0.0,0.0,0.0};
    std::vector<double> twist = {0.0,0.0,0.0,0.0,0.0,0.0};
    std::vector<double> jnts_q, jnts_v, jnts_a, jnts_eff;

    std::string base_link_name = "base_link";
    
    std::vector<std::string> dof_names;
    int n_dofs=-1;

    int step_counter=0;
    double physics_dt=-1.0;
    
    double rt_factor_trgt;
    double step_dt_trgt_walltime;

private:

    bool headless;
    bool manual_stepping;

    bool match_rt_factor;
    double rt_factor_dt = 0.01; // [s]

    std::atomic_bool running=false;
    std::atomic_bool initialized = false;
    int steps_counter = 0;

    int init_steps = 1;

    // cpu-sim syncronization points
    double cpusync = 0;
    mjtNum simsync = 0;

    std::string model_fname; 
    std::string xbot2_config_path;

    bool render_to_file;
    std::string custom_camera_name;
    mjvCamera custom_mj_cam;
    int custom_cam_width = 1920;
    int custom_cam_height = 1080;
    std::string render_base_path, render_path;
    std::atomic_bool custom_camera_ready=false;

    float* depth = nullptr;
    // Define the rectangle to read pixels from
    mjrRect custom_cam_rect;
    float render_fps;
    int render_phstepfreq;

    std::thread physics_thread;
    std::thread simulator_thread;

    std::condition_variable sim_step_req_cv, sim_step_res_cv;
    bool step_req = false;
    bool step_done = true;
    std::mutex ph_step_mtx; 
    int timeout = 5; // [s]
    
    std::condition_variable render_step_req_cv, render_step_res_cv;
    std::mutex render_mtx; 
    bool render_req = false;
    bool render_done = false;

    std::chrono::time_point<mj::Simulate::Clock> syncCPU;
    mjtNum syncSim = 0;

    void initialize(bool headless);
    void physics_loop();
    void physics_loop_manual();
    void step_sim();
    void clear_sim();
    void assign_init_root_state();
    bool run();
    void read_dofs();
    void read_root();
    
    void init_custom_camera();
    void render_png(int frame_idx = 0);

    void rendering_loop();

};

#endif // XMJ_SIM_ENV_H
