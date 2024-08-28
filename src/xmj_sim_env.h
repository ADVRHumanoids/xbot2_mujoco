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

namespace mj = ::mujoco;

class XBotMjSimEnv {
public:

    typedef std::unique_ptr<XBotMjSimEnv> UniquePtr;

    XBotMjSimEnv(
        const std::string model_fname,
        ros::NodeHandle nh,
        const std::string xbot2_config_path = "",
        bool headless = false,
        bool manual_stepping = false,
        int init_steps = 1,
        int timeout = 10);
    ~XBotMjSimEnv();

    bool is_running();
    bool step();
    bool reset();
    void close();

    std::vector<double> p_i = {0.0,0.0,0.8};
    std::vector<double> q_i = {1.0, 0.0, 0.0, 0.0};
    std::string base_link_name = "base_link";

    int step_counter=0;
    double physics_dt=-1.0;

private:

    bool headless;
    bool manual_stepping;

    std::atomic_bool running=false;
    std::atomic_bool initialized = false;

    int init_steps = 1;
    
    // cpu-sim syncronization points
    double cpusync = 0;
    mjtNum simsync = 0;

    std::string model_fname; 
    std::string xbot2_config_path;

    std::thread physics_thread;
    std::thread simulator_thread;

    std::condition_variable sim_step_req_cv, sim_step_res_cv;
    bool step_req = false;
    bool step_done = true;
    std::mutex mtx; 
    int timeout = 1; // [s]

    ros::NodeHandle ros_nh;
    
    std::chrono::time_point<mj::Simulate::Clock> syncCPU;
    mjtNum syncSim = 0;

    void initialize(bool headless);
    void physics_loop();
    void physics_loop_manual();
    void step_sim();
    void clear_sim();
    void assign_init_root_state();
    bool run();

};

#endif // XMJ_SIM_ENV_H
