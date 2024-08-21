#include "xmj_sim_env.h"


XBotMjSimEnv::XBotMjSimEnv(const std::string configPath, 
    const std::string model_fname,
    ros::NodeHandle nh,
    bool headless,
    bool manual_stepping)
    :xbot2_cfg_path(configPath),model_fname(model_fname),nh(nh),headless(headless),manual_stepping(manual_stepping) {

    printf("[xbot2_mujoco][XBotMjSimEnv]: initializing sim. enviroment with MuJoCo xml file at %s and XBot2 config at %s\n", 
        model_fname.c_str(), xbot2_cfg_path.c_str());

}

XBotMjSimEnv::~XBotMjSimEnv() {

    close();

}

void XBotMjSimEnv::close() {

    if (!running) {
        
        running=false;
    }
}

void XBotMjSimEnv::reset() {

}

void XBotMjSimEnv::step() {

}

void XBotMjSimEnv::render_window() {

}

void XBotMjSimEnv::run() {

    if (!running) {
        if (!manual_stepping) {
            xbot_mujoco::run(model_fname.c_str(),xbot2_cfg_path,nh,headless); // sim in separate thread and rendering loop
        } {
            if (!headless) {
                // spawn rendering thread
            }
        }

        running=true;
    }
    
}



