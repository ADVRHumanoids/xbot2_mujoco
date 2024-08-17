#ifndef XMJ_SIM_ENV_H
#define XMJ_SIM_ENV_H

#include "stdio.h"
#include "string.h"

#include <thread>
#include <mutex>
#include <chrono>
#include <memory>

#include "simulator.h"

class XBotMjSimEnv {
public:

    typedef std::unique_ptr<XBotMjSimEnv> UniquePtr;

    XBotMjSimEnv(const std::string xbot2_cfg_path,
        const std::string model_fname = "",
        bool headless = false,
        bool manual_stepping = false);
    ~XBotMjSimEnv();

    void step();
    void render_window();
    void reset();
    void close();
    void run(ros::NodeHandle nh);

private:

    bool headless;
    bool manual_stepping;
    bool running=true;
    
    int sim_init_steps = 0;

    // cpu-sim syncronization points
    double cpusync = 0;
    mjtNum simsync = 0;

    std::string model_fname; 
    std::string xbot2_cfg_path;

    std::thread rendering_thread;

};

#endif // XMJ_SIM_ENV_H
