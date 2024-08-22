#ifndef XMJ_SIM_ENV_H
#define XMJ_SIM_ENV_H

#include "stdio.h"
#include "string.h"

#include <thread>
#include <mutex>
#include <chrono>
#include <memory>

#include "simulator.h"

namespace mj = ::mujoco;

class XBotMjSimEnv {
public:

    typedef std::unique_ptr<XBotMjSimEnv> UniquePtr;

    XBotMjSimEnv(const std::string xbot2_config_path,
        const std::string model_fname,
        ros::NodeHandle nh,
        bool headless = false,
        bool manual_stepping = false,
        int init_steps = 1);
    ~XBotMjSimEnv();

    bool run(); 
    
    void render_window();
    void reset(std::vector<double> p, std::vector<double> q,
        std::string base_link_name="base_link");
    void close();

private:

    bool headless;
    bool manual_stepping;
    bool running=false;
    
    int sim_init_steps = 0;
    int init_steps = 1;
    
    // cpu-sim syncronization points
    double cpusync = 0;
    mjtNum simsync = 0;

    std::string model_fname; 
    std::string xbot2_config_path;

    std::thread physics_thread;

    ros::NodeHandle ros_nh;
    
    std::chrono::time_point<mj::Simulate::Clock> syncCPU;
    mjtNum syncSim = 0;

    void initialize(bool headless);
    void physics_loop();
    void step();

};

#endif // XMJ_SIM_ENV_H
