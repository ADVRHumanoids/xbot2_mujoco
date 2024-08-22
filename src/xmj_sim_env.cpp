#include "xmj_sim_env.h"
#include <csignal>  // For signal handling

void handle_sigint(int signal_num) {
  std::printf("[xbot2_mujoco][XBotMjSimEnv]: detected SIGINT -> exiting gracefully \n");
  xbot_mujoco::sim->exitrequest.store(1); // signal sim ad rendering loop to exit gracefully
}

XBotMjSimEnv::XBotMjSimEnv(const std::string configPath, 
    const std::string model_fname,
    ros::NodeHandle nh,
    bool headless,
    bool manual_stepping,
    int init_steps,
    int timeout)
    :xbot2_config_path(configPath),model_fname(model_fname),ros_nh(nh),
    headless(headless),manual_stepping(manual_stepping), timeout(timeout) {

    printf("[xbot2_mujoco][XBotMjSimEnv]: initializing sim. enviroment with MuJoCo xml file at %s and XBot2 config at %s\n", 
        model_fname.c_str(), xbot2_config_path.c_str());

}

XBotMjSimEnv::~XBotMjSimEnv() {

    close();

}

void XBotMjSimEnv::close() {

    if (running) {

        running=false;
    }
}

bool XBotMjSimEnv::step() {

    if (manual_stepping) {
        std::lock_guard<std::mutex> lock(mtx);
        step_now = true;    
        sim_step_cv.notify_one();
        return true;
    } else {
        return false;
    }
}

void XBotMjSimEnv::assign_init_root_state() {

    xbot_mujoco::p_init.assign(p_i.begin(),p_i.end());
    xbot_mujoco::q_init.assign(q_i.begin(),q_i.end());
    xbot_mujoco::root_link=base_link_name;

}

void XBotMjSimEnv::reset() {
    
    auto& simulation = *xbot_mujoco::sim;
    xbot_mujoco::Reset(simulation);
}

void XBotMjSimEnv::clear_sim() {
    xbot_mujoco::ClearSimulation();
    xbot_mujoco::xbot2_wrapper.reset();
}

void XBotMjSimEnv::physics_loop() {
    // InitSimulation has to be called here since it will wait for the render thread to
    // finish loading
    xbot_mujoco::InitSimulation(xbot_mujoco::sim.get(),model_fname.c_str(),xbot2_config_path.c_str());
    reset();

    initialized = true;

    while (!((*xbot_mujoco::sim).exitrequest.load())) {
        step_sim();
    }
    
    clear_sim();

}

void XBotMjSimEnv::physics_loop_manual() {
    // InitSimulation has to be called here since it will wait for the render thread to
    // finish loading
    xbot_mujoco::InitSimulation(xbot_mujoco::sim.get(),model_fname.c_str(),xbot2_config_path.c_str());
    reset();

    initialized = true;

    while (true) {

        std::unique_lock<std::mutex> lock(mtx);  // Lock the mutex
        if (!sim_step_cv.wait_for(lock, std::chrono::seconds(timeout), [this] { return step_now; })) {
            printf("[xbot2_mujoco][XBotMjSimEnv][physics_loop_manual]: no step request received within timeout of %i s\n", timeout);
            xbot_mujoco::sim->exitrequest.store(1);
            clear_sim();
            return;
        } 

        if (!((*xbot_mujoco::sim).exitrequest.load())) {
            step_sim();
            step_now=false; // stepped simulation
        } else {
            clear_sim();
            return;
        }
        
    }

}

void XBotMjSimEnv::step_sim() {

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
    std::printf("[XBotMjSimEnv][initialize]: MuJoCo version %s\n", mj_versionString());
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
    
    assign_init_root_state();

    // Install the signal handler for SIGINT (Ctrl+C)
    std::signal(SIGINT, handle_sigint);
    // physics_thread = std::thread(&xbot_mujoco::SimulationLoop, xbot_mujoco::sim.get(), 
    //     model_fname.c_str(), xbot2_config_path.c_str());
    if (!manual_stepping) {
        printf("[xbot2_mujoco][XBotMjSimEnv][initialize]: launching physics sim thread\n");
        physics_thread = std::thread(&XBotMjSimEnv::physics_loop, this);
    } else { // will (atomically) wait for step to be called
        printf("[xbot2_mujoco][XBotMjSimEnv][initialize]: launching physics sim threadh with manual stepping\n");
        physics_thread = std::thread(&XBotMjSimEnv::physics_loop_manual, this);
    }

    xbot_mujoco::RenderingLoop(xbot_mujoco::sim.get(), ros_nh); // render in this thread
    
    if (physics_thread.joinable()) {
        physics_thread.join();
        printf("[xbot2_mujoco][XBotMjSimEnv][close]: physics thread terminated\n");
    }

}

bool XBotMjSimEnv::run() {

    if (!running) {
        simulator_thread = std::thread(&XBotMjSimEnv::initialize, this, headless);

        if (simulator_thread.joinable()) { 
            simulator_thread.join();
            printf("[xbot2_mujoco][XBotMjSimEnv][close]: simulator thread terminated\n");
        }
        
        running=true;
        return true;
    } else {
        return false;
    }
    
}



