#include "xmj_sim_env.h"


XBotMjSimEnv::XBotMjSimEnv(const std::string configPath, 
    const char* model_fname,
    bool headless,
    bool multithread)
    :xbot2_cfg_path(configPath),headless(headless),multithread(multithread) {
    
    mju_strncpy(filename, model_fname, 1000);

    initialize();

}

XBotMjSimEnv::~XBotMjSimEnv() {

    close();

}

void XBotMjSimEnv::close() {

    if (!closed) {
        require_exit();
        if (rendering_thread.joinable()) {
            rendering_thread.join();
        }

        xbot2_wrapper.reset();

        fprintf(stderr, "[XBotMjSimEnv][close]: destroyed xbot2 wrapper. \n");

        // delete everything we allocated
        mj_deleteData(d);
        mj_deleteModel(m);
        mjv_freeScene(&scn);
        mjr_freeContext(&con);
        if (!headless) {
            uiClearCallback(window);
            // terminate GLFW (crashes with Linux NVidia drivers)
            #if defined(__APPLE__) || defined(_WIN32)
                glfwTerminate();
            #endif
        }

        fprintf(stderr, "[XBotMjSimEnv][close]: finished simulation cleanup. \n");

        closed=true;
    }
}

void XBotMjSimEnv::initialize() {

    closed=false;

    init(headless);

    cpusync = 0;
    simsync = 0;

    if (!headless && multithread) { // rendering in separate thread

        // do some simulation stepping
        // for (int i = 0; i < sim_init_steps; i++) {
        //     step(cpusync,simsync);
        // }

        // spawn rendering in separate thread
        rendering_thread = std::thread(&XBotMjSimEnv::launch_rendering_loop, this);

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
            loadmodel();
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