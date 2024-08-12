#include "xmj_sim_env.h"

XBotMjSimEnv::XBotMjSimEnv(const std::string configPath) : xbot2_cfg_path(configPath) {
    
}

XBotMjSimEnv::~XBotMjSimEnv() {

}

void XBotMjSimEnv::run() {

}

void XBotMjSimEnv::step() {

}

void XBotMjSimEnv::render() {
    if(!headless) {
        int a = 1;
    }
}

void XBotMjSimEnv::close() {

}