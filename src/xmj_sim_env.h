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
    XBotMjSimEnv(const std::string xbot2_cfg_path);
    ~XBotMjSimEnv();

    void run();
    void step();
    void render();
    void reset();
    void close();

private:

    bool headless=true;

    char filename[1000] = "";
    std::string xbot2_cfg_path;

};

#endif // XMJ_SIM_ENV_H
