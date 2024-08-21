#include "xmj_sim_env.h"

XBotMjSimEnv::XBotMjSimEnv(const std::string configPath, 
    const std::string model_fname,
    ros::NodeHandle nh,
    bool headless,
    bool manual_stepping,
    int init_steps)
    :xbot2_cfg_path(configPath),model_fname(model_fname),ros_nh(nh),headless(headless),manual_stepping(manual_stepping) {

    printf("[xbot2_mujoco][XBotMjSimEnv]: initializing sim. enviroment with MuJoCo xml file at %s and XBot2 config at %s\n", 
        model_fname.c_str(), xbot2_cfg_path.c_str());

}

XBotMjSimEnv::~XBotMjSimEnv() {

    close();

}

void XBotMjSimEnv::close() {

    if (!running) {

        if (!manual_stepping) {
            xbot_mujoco::ClearSimulation();
            xbot_mujoco::xbot2_wrapper.reset();
        }

        running=false;
    }
}

void XBotMjSimEnv::reset(std::vector<double> p, std::vector<double> q,
    std::string base_link_name) {
    xbot_mujoco::root_link=base_link_name;
    xbot_mujoco::p_init.assign(p.begin(), p.end());
    xbot_mujoco::q_init.assign(q.begin(), q.end());
    auto& simulation = *xbot_mujoco::sim;
    xbot_mujoco::Reset(simulation);
}

bool XBotMjSimEnv::step() {
    if ((*xbot_mujoco::sim).exitrequest.load()) {
        return false;
    }

    xbot_mujoco::PreStep(*xbot_mujoco::sim);
    xbot_mujoco::DoStep(*xbot_mujoco::sim,syncCPU,syncSim);

    return true;
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
    
    xbot_mujoco::xbot2_cfg_path=this->xbot2_cfg_path;
    
    xbot_mujoco::InitSimulation(xbot_mujoco::sim.get(),model_fname.c_str());
    
    xbot_mujoco::RenderingLoop(xbot_mujoco::sim.get(), ros_nh); // render in this thread

    // do some warmstart timesteps
    bool init_step_ok = true;
    for (int i=0; i < init_steps;i++) {
        init_step_ok = step();
    }

}

void XBotMjSimEnv::run() {

    if (!running) {
        if (!manual_stepping) {
            xbot_mujoco::run(model_fname.c_str(),xbot2_cfg_path,ros_nh,headless); // sim in separate thread and rendering loop
        } else { // manual stepping setup
            initialize(headless);
        }

        running=true;
    }
    
}



