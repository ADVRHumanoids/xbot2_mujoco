#ifndef XMJ_SIM_ENV_H
#define XMJ_SIM_ENV_H

#include "stdio.h"
#include "string.h"

#include <thread>
#include <mutex>
#include <chrono>

#include "simulator.h"

class XBotMjSimEnv {
public:
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

    int sim_init_steps = 0;

    // cpu-sim syncronization points
    double cpusync = 0;
    mjtNum simsync = 0;

    char model_fname[1000];
    std::string xbot2_cfg_path;

    std::thread rendering_thread;

};

#endif // XMJ_SIM_ENV_H
