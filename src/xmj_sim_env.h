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
        const char* model_fname = "",
        bool headless = false,
        bool multithread = true);
    ~XBotMjSimEnv();

    void step();
    void render_window();
    void reset();
    void close();

private:

    void initialize();

    void launch_rendering_loop();

    bool headless;
    bool multithread;
    bool closed=false;
    
    int sim_init_steps = 0;

    // cpu-sim syncronization points
    double cpusync = 0;
    mjtNum simsync = 0;

    char xml_fname[1000];
    std::string xbot2_cfg_path;

    std::thread rendering_thread;

};

#endif // XMJ_SIM_ENV_H
