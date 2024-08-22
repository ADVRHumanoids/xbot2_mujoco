#include "xmj_sim_env.h"
#include <csignal>  // For signal handling

void handle_sigint(int signal_num) {
  std::printf("[xbot2_mujoco][simulator]: detected SIGINT -> exiting gracefully \n");
  xbot_mujoco::sim->exitrequest.store(1); // signal sim ad rendering loop to exit gracefully
}

XBotMjSimEnv::XBotMjSimEnv(const std::string configPath, 
    const std::string model_fname,
    ros::NodeHandle nh,
    bool headless,
    bool manual_stepping,
    int init_steps)
    :xbot2_config_path(configPath),model_fname(model_fname),ros_nh(nh),headless(headless),manual_stepping(manual_stepping) {

    printf("[xbot2_mujoco][XBotMjSimEnv]: initializing sim. enviroment with MuJoCo xml file at %s and XBot2 config at %s\n", 
        model_fname.c_str(), xbot2_config_path.c_str());

}

XBotMjSimEnv::~XBotMjSimEnv() {

    close();

}

void XBotMjSimEnv::close() {

    if (running) {

        xbot_mujoco::ClearSimulation();
        xbot_mujoco::xbot2_wrapper.reset();
        
        if (physics_thread.joinable()) {
            physics_thread.join();
            printf("[xbot2_mujoco][XBotMjSimEnv][close]: physics thread terminated\n");
        }

        running=false;
    }
}

void XBotMjSimEnv::assign_init_root_state() {

    auto& simulation = *xbot_mujoco::sim;
    xbot_mujoco::Reset(simulation, p_i, q_i, base_link_name);

}

void XBotMjSimEnv::reset() {
    
    assign_init_root_state(); // in case p_i and q_i were changed
    auto& simulation = *xbot_mujoco::sim;
    xbot_mujoco::Reset(simulation, p_i, q_i, base_link_name);
}

void XBotMjSimEnv::physics_loop() {
    // InitSimulation has to be called here since it will wait for the render thread to
    // finish loading
    xbot_mujoco::InitSimulation(xbot_mujoco::sim.get(),model_fname.c_str(),xbot2_config_path.c_str());
    
    reset();

    while (!((*xbot_mujoco::sim).exitrequest.load())) {
        step();
    }
}

void XBotMjSimEnv::step() {

    xbot_mujoco::PreStep(*xbot_mujoco::sim);
    xbot_mujoco::DoStep(*xbot_mujoco::sim,syncCPU,syncSim);

}

void XBotMjSimEnv::render_window() {

}

void XBotMjSimEnv::initialize(bool headless) {
    // display an error if running on macOS under Rosetta 2
    #if defined(__APPLE__) && defined(__AVX__)
    if (rosetta_error_msg) {
        DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
        std::exit(1);
    }
    #endif

    // print version, check compatibility
    std::printf("[xbot2_mujoco][simulator]: MuJoCo version %s\n", mj_versionString());
    if (mjVERSION_HEADER!=mj_version()) {
        mju_error("Headers and library have different versions");
    }

    // scan for libraries in the plugin directory to load additional plugins
    xbot_mujoco::scanPluginLibraries();

    mjvCamera cam;
    mjv_defaultCamera(&cam);

    mjvOption opt;
    mjv_defaultOption(&opt);

    mjvPerturb pert;
    mjv_defaultPerturb(&pert);

    // simulate object encapsulates the UI
    xbot_mujoco::sim = std::make_unique<mj::Simulate>(
        std::make_unique<mj::GlfwAdapter>(),
        &cam, &opt, &pert, /* is_passive = */ false
    );

    mjcb_control = xbot_mujoco::xbotmj_control_callback; // register control callback to mujoco
    
    if ((!headless) && (!manual_stepping)) {
        // Install the signal handler for SIGINT (Ctrl+C)
        std::signal(SIGINT, handle_sigint);
        // physics_thread = std::thread(&xbot_mujoco::SimulationLoop, xbot_mujoco::sim.get(), 
        //     model_fname.c_str(), xbot2_config_path.c_str());
        physics_thread = std::thread(&XBotMjSimEnv::physics_loop, this);
        xbot_mujoco::RenderingLoop(xbot_mujoco::sim.get(), ros_nh); // render in this thread

    }

    if ((headless) && (!manual_stepping)) {

    }

    if ((!headless) && (manual_stepping)) {

    }

    if ((headless) && (manual_stepping)) {

    }


    // do some warmstart timesteps
    bool init_step_ok = true;
    for (int i=0; i < init_steps;i++) {
        step();
    }

}

bool XBotMjSimEnv::run() {

    if (!running) {
        initialize(headless);
        running=true;
        return true;
    } else {
        return false;
    }
    
}



