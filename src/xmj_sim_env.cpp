#include "xmj_sim_env.h"


XBotMjSimEnv::XBotMjSimEnv(const std::string configPath, 
    const std::string model_fname,
    bool headless,
    bool manual_stepping)
    :xbot2_cfg_path(configPath),model_fname(model_fname),headless(headless),manual_stepping(manual_stepping) {

    printf( "[xbot2_mujoco][XBotMjSimEnv]: will use MuJoCo xml file at %s and XBot2 config at %s\n", model_fname.c_str(), xbot2_cfg_path.c_str());

    initialize();

}

XBotMjSimEnv::~XBotMjSimEnv() {

    close();

}

void XBotMjSimEnv::close() {

    if (!running) {
        
        running=false;
    }
}

void XBotMjSimEnv::run(ros::NodeHandle nh) {
    
    if (!running) {
        if (!manual_stepping) {
            run(model_fname.c_str(),xbot2_cfg_path,nh,headless); // sim in separate thread and rendering loop
        } {
            if (!headless) {
                // spawn rendering thread
            }
        }

        running=true;
    }
    
}

void XBotMjSimEnv::reset() {

}

void XBotMjSimEnv::launch_rendering_loop() {

    rendering_loop(headless);

}

void XBotMjSimEnv::step() {

    do_step(cpusync,simsync);
}

void XBotMjSimEnv::render_window() {
    
    if(!headless && !multithread &&
        !settings.exitrequest && !glfwWindowShouldClose(window)) {

        // start exclusive access (block simulation thread)
        mtx.lock();

        // load model (not on first pass, to show "loading" label)
        if( settings.loadrequest==1 )
            loadmodel(xml_fname);
        else if( settings.loadrequest>1 )
            settings.loadrequest = 1;
        
        // handle events (calls all callbacks)
        glfwPollEvents();

        // prepare to render
        prepare();

        // end exclusive access (allow simulation thread to run)
        mtx.unlock();

        // render while simulation is running
        render(window);

    }
}