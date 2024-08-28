#include "xmj_sim_env.h"
#include <csignal>  // For signal handling

void handle_sigint(int signal_num) {
  std::printf("[xbot2_mujoco][XBotMjSimEnv]: detected SIGINT -> exiting gracefully \n");
  xbot_mujoco::sim->exitrequest.store(1); // signal sim ad rendering loop to exit gracefully
}

XBotMjSimEnv::XBotMjSimEnv(
    const std::string model_fname,
    ros::NodeHandle nh,
    const std::string xbot2_config_path, 
    bool headless,
    bool manual_stepping,
    int init_steps,
    int timeout)
    :xbot2_config_path(xbot2_config_path),model_fname(model_fname),ros_nh(nh),
    headless(headless),manual_stepping(manual_stepping), timeout(timeout) {

    printf("[xbot2_mujoco][XBotMjSimEnv]: initializing sim. enviroment with MuJoCo xml file at %s and XBot2 config at %s\n", 
        model_fname.c_str(), xbot2_config_path.c_str());

    if (!run()) {
        fprintf(stderr, "[xbot2_mujoco][XBotMjSimEnv]: failed to run environment!");
    }

}

XBotMjSimEnv::~XBotMjSimEnv() {

    close();

}

void XBotMjSimEnv::close() {

    if (running.load()) {
        
        if (simulator_thread.joinable()) { 
            
            simulator_thread.join();
            printf("[xbot2_mujoco][XBotMjSimEnv][close]: simulator thread terminated\n");
        }
        
        running.store(false);
        initialized.store(false);
    }
}

bool XBotMjSimEnv::step() {

    if (manual_stepping && running.load()) {
        std::unique_lock<std::mutex> lock(mtx);
        step_req = true;  
        step_done=false;  
        sim_step_req_cv.notify_all(); // send step request
        // wait for ack from simulation loop (will set step_req to false
        // sim_step_res_cv.wait(lock, [this] { return !step_done; });
        if (!sim_step_res_cv.wait_for(lock, std::chrono::milliseconds(timeout), [this] { return step_done; })) {
            fprintf(stderr,"[xbot2_mujoco][XBotMjSimEnv][step]: no step acknowledgement received within timeout of %i ms\n", timeout);
            xbot_mujoco::sim->exitrequest.store(1);
            return false;
        } // sim step performed successfully

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

bool XBotMjSimEnv::reset() {
    
    if (is_running()) {
        auto& simulation = *xbot_mujoco::sim;
        assign_init_root_state();
        xbot_mujoco::Reset(simulation);
        return true;
    } else {
        return false;
    }
    
}

void XBotMjSimEnv::clear_sim() {
    xbot_mujoco::ClearSimulation();
    xbot_mujoco::xbot2_wrapper.reset();
}

void XBotMjSimEnv::physics_loop() {
    // InitSimulation has to be called here since it will wait for the render thread to
    // finish loading

    std::unique_lock<std::mutex> lock(mtx);

    xbot_mujoco::InitSimulation(xbot_mujoco::sim.get(),model_fname.c_str(),xbot2_config_path.c_str());
    reset();
    physics_dt=xbot_mujoco::m->opt.timestep;

    initialized.store(true);

    lock.unlock();

    while (!((*xbot_mujoco::sim).exitrequest.load())) {
        step_sim();
    }
    
}

void XBotMjSimEnv::physics_loop_manual() {
    // InitSimulation has to be called here since it will wait for the render thread to
    // finish loading

    std::unique_lock<std::mutex> lock(mtx);

    xbot_mujoco::InitSimulation(xbot_mujoco::sim.get(),model_fname.c_str(),xbot2_config_path.c_str());
    reset();
    physics_dt=xbot_mujoco::m->opt.timestep;

    initialized.store(true);

    lock.unlock();

    while (!(xbot_mujoco::sim->exitrequest.load())) {

        std::unique_lock<std::mutex> lock(mtx); 
        // sim_step_req_cv.wait(lock, [this] { return step_req; });
        if (!sim_step_req_cv.wait_for(lock, std::chrono::milliseconds(timeout), [this] { return step_req; })) {
            printf("[xbot2_mujoco][XBotMjSimEnv][physics_loop_manual]: no step request received within timeout of %i ms\n", timeout);
            xbot_mujoco::sim->exitrequest.store(1);
            return;
        } 
        step_sim();
        step_req=false;
        step_done=true; // signal that simulation has stepped
        sim_step_res_cv.notify_all();
    }
    
}

void XBotMjSimEnv::step_sim() {

    xbot_mujoco::PreStep(*xbot_mujoco::sim);
    xbot_mujoco::DoStep(*xbot_mujoco::sim,syncCPU,syncSim);
    step_counter=xbot_mujoco::step_counter;

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
        printf("[xbot2_mujoco][XBotMjSimEnv][initialize]: launching physics sim thread with manual stepping\n");
        physics_thread = std::thread(&XBotMjSimEnv::physics_loop_manual, this);
    }

    xbot_mujoco::RenderingLoop(xbot_mujoco::sim.get(), ros_nh); // render in this thread
    
    if (physics_thread.joinable()) {
        physics_thread.join();
        printf("[xbot2_mujoco][XBotMjSimEnv][close]: physics thread terminated\n");
    }

    clear_sim();

}

bool XBotMjSimEnv::is_running() {
    return running.load();
}

bool XBotMjSimEnv::run() {

    if (!is_running()) {

        simulator_thread = std::thread(&XBotMjSimEnv::initialize, this, headless);

        while (!initialized.load()) {
            std::this_thread::sleep_for(100ms);
        }

        running.store(true);

        if (manual_stepping) { // perform some dummy sim steps
            for (int i = 0; i < init_steps; ++i) {
                if (!step()) {
                    return false;
                }
            }
        } 

        return true;

    } else {

        return false;
        
    }
    
}



