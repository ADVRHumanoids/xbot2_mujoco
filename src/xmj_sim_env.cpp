#include "xmj_sim_env.h"
#include <csignal>  // For signal handling

void handle_sigint(int signal_num) {
  std::printf("[xbot2_mujoco][XBotMjSimEnv]: detected SIGINT -> exiting gracefully \n");
  xbot_mujoco::sim->exitrequest.store(1); // signal sim ad rendering loop to exit gracefully
}

XBotMjSimEnv::XBotMjSimEnv(
    const std::string model_fname,
    const std::string xbot2_config_path, 
    bool headless,
    bool manual_stepping,
    int init_steps,
    int timeout)
    :xbot2_config_path(xbot2_config_path),model_fname(model_fname),
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
        
        read_state(); // update state from sim

        return true;
    } else { 
        return false;
    }
}

void XBotMjSimEnv::read_state() {
    read_dofs();
    read_root();
}

void XBotMjSimEnv::read_dofs() {
    // we should be ok (lengths match)
    // std::copy(d->qpos, d->qpos + n_dofs, jnts_q.begin());
    // std::copy(d->qvel, d->qvel + n_dofs, jnts_v.begin());
    // std::copy(d->qacc, d->qacc + n_dofs, jnts_a.begin());
    // std::copy(d->qfrc_applied, d->qfrc_applied + n_dofs, jnts_eff.begin());
    for (int i=0; i<dof_names.size(); i++) {
        int joint_id = mj_name2id(xbot_mujoco::m, mjOBJ_JOINT, dof_names[i].c_str());
        int qi = xbot_mujoco::m->jnt_qposadr[joint_id];
        int vi = xbot_mujoco::m->jnt_dofadr[joint_id];
        jnts_q[i]=xbot_mujoco::d->qpos[qi];
        jnts_v[i]=xbot_mujoco::d->qvel[vi];
        jnts_a[i]=xbot_mujoco::d->qacc[vi];
        jnts_eff[i]=xbot_mujoco::d->qfrc_applied[vi];
    }
}

void XBotMjSimEnv::read_root() {
    int rott_link_idx = mj_name2id(xbot_mujoco::m, mjOBJ_BODY, base_link_name.c_str());
    if (rott_link_idx != -1) { // root link found
        int root_pos_address = xbot_mujoco::m->jnt_qposadr[xbot_mujoco::m->body_jntadr[rott_link_idx]];
        int root_vel_address = xbot_mujoco::m->jnt_dofadr[xbot_mujoco::m->body_jntadr[rott_link_idx]];
        std::copy(xbot_mujoco::d->qpos+root_pos_address, 
            xbot_mujoco::d->qpos+root_pos_address+3, p.begin()); // root position
        std::copy(xbot_mujoco::d->qpos+root_pos_address+3, 
            xbot_mujoco::d->qpos+root_pos_address+7, q.begin()); // root orientation
        std::copy(xbot_mujoco::d->qvel+root_vel_address, 
            xbot_mujoco::d->qvel+root_vel_address+6, twist.begin()); // root twist

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

    n_dofs = n_jnts();
    jnts_q.resize(n_dofs);
    jnts_v.resize(n_dofs);
    jnts_a.resize(n_dofs);
    jnts_eff.resize(n_dofs);

    dof_names.resize(n_dofs);
    dof_names=jnt_names();

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

    n_dofs = n_jnts();
    jnts_q.resize(n_dofs);
    jnts_v.resize(n_dofs);
    jnts_a.resize(n_dofs);
    jnts_eff.resize(n_dofs);

    dof_names.resize(n_dofs);
    dof_names=jnt_names();

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
    // ROS stuff

    // std::string ros_namespace=""; // operate in ros global ns
    // ros::NodeHandle ros_nh(ros_namespace);

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

    // xbot_mujoco::RenderingLoop(xbot_mujoco::sim.get(), ros_nh); // render in this thread
    xbot_mujoco::RenderingLoop(xbot_mujoco::sim.get()); // render in this thread

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

        // std::vector<std::string> args;
        // std::vector<char*> argv;
        // for (std::string& arg : args)
        // {
        //     argv.push_back(&arg[0]);
        // }
        // int argc = argv.size();
        // ros::init(argc, argv.data(), std::string("XBotMjSimEnv"));

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

std::vector<std::string> XBotMjSimEnv::jnt_names() {
    return xbot_mujoco::JntNames(xbot_mujoco::m);
}

int XBotMjSimEnv::n_jnts() {
    std::vector<std::string> names=xbot_mujoco::JntNames(xbot_mujoco::m);
    return names.size();
}

void XBotMjSimEnv::move_to_homing_now() {
    xbot_mujoco::MoveJntsToHomingNow(xbot_mujoco::d);
}

void XBotMjSimEnv::move_base_to_now(std::vector<double> p, std::vector<double> q) {
    xbot_mujoco::MoveBaseNowTo(xbot_mujoco::d, p, q, base_link_name);
}