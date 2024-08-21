#include "xmj_sim_env.h"

namespace mj = ::mujoco;

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
        
        xbot_mujoco::ClearSimulation();
        xbot_mujoco::xbot2_wrapper.reset();

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

void XBotMjSimEnv::step() {

}

void XBotMjSimEnv::render_window() {

}

void XBotMjSimEnv::initialize() {
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

    // RenderingLoop(sim.get(), nh); // render in this thread

}

void XBotMjSimEnv::run() {

    if (!running) {
        if (!manual_stepping) {
            xbot_mujoco::run(model_fname.c_str(),xbot2_cfg_path,nh,headless); // sim in separate thread and rendering loop
        } else { // manual stepping setup
            if (!headless) {
                // spawn rendering thread
            }
        }

        running=true;
    }
    
}



