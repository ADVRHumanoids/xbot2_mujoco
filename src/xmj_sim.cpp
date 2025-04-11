#include "xmj_sim.h"
#include <csignal>  // For signal handling
#include <cmath>
#include <time.h>
#include "lodepng.h"
#include <filesystem>

void highResolutionSleep(std::chrono::nanoseconds sleepDuration) {
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = sleepDuration.count();

    while (clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, &ts) == EINTR) {
        // Handle interruptions if necessary
    }
}

void handle_sigint(int signal_num) {
  std::printf("[xbot2_mujoco][XBotMjSim]: detected SIGINT -> exiting gracefully \n");
  xbot_mujoco::sim->exitrequest.store(1); // signal sim ad rendering loop to exit gracefully
}

std::string get_current_time_as_string() {
    // Get current time as time_t
    auto now = std::chrono::system_clock::now();
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    
    // Convert to tm structure
    std::tm tm = *std::localtime(&now_time_t);

    // Create a string stream to format the date and time
    std::ostringstream oss;
    oss << std::put_time(&tm, "%H_%M_%S_%d_%m_%Y");  // Format: hh_mm_ss_DD_MM_YYYY
    return oss.str();
}

XBotMjSim::XBotMjSim(
    const std::string model_fname,
    const std::string xbot2_config_path, 
    bool headless,
    bool manual_stepping,
    int init_steps,
    int timeout,
    const std::string base_link_name,
    bool match_rt_factor,
    double rt_factor_trgt,
    bool render_to_file,
    std::string custom_camera_name,
    std::string render_base_path,
    float render_fps)
    :xbot2_config_path(xbot2_config_path),model_fname(model_fname),
    headless(headless),manual_stepping(manual_stepping), timeout(timeout),
    base_link_name(base_link_name),
    match_rt_factor(match_rt_factor),
    rt_factor_trgt(rt_factor_trgt),
    render_to_file(render_to_file),
    custom_camera_name(custom_camera_name),
    render_base_path(render_base_path),
    render_fps(render_fps) {
    
    printf("[xbot2_mujoco][XBotMjSim]: initializing sim. enviroment with MuJoCo xml file at %s and XBot2 config at %s, base link name %s, match_rt_factor %d, rt_factor_trgt: %f\n", 
        model_fname.c_str(), xbot2_config_path.c_str(), base_link_name.c_str(), match_rt_factor, rt_factor_trgt);

    if (!manual_stepping && match_rt_factor) {
        printf("[xbot2_mujoco][XBotMjSim]: won't be able to match desired rt since manual_stepping is set to false!");
    }

    if (!run()) {
        fprintf(stderr, "[xbot2_mujoco][XBotMjSim]: failed to run environment!");
    }
    

}

XBotMjSim::~XBotMjSim() {

    close();

}

void XBotMjSim::close() {

    if (running.load()) {
        
        if (simulator_thread.joinable()) { 
            
            simulator_thread.join();
            printf("[xbot2_mujoco][XBotMjSim][close]: simulator thread terminated\n");
        }
        
        running.store(false);
        initialized.store(false);
    }
}

bool XBotMjSim::step() {

    if (manual_stepping && running.load()) {
        std::unique_lock<std::mutex> lock(ph_step_mtx);
        step_req = true;  
        step_done=false;  
        sim_step_req_cv.notify_all(); // send step request
        // wait for ack from simulation loop (will set step_req to false
        // sim_step_res_cv.wait(lock, [this] { return !step_done; });
        if (!sim_step_res_cv.wait_for(lock, std::chrono::milliseconds(timeout), [this] { return step_done; })) {
            fprintf(stderr,"[xbot2_mujoco][XBotMjSim][step]: no step acknowledgement received within timeout of %i ms\n", timeout);
            xbot_mujoco::sim->exitrequest.store(1);
            return false;
        } // sim step performed successfully
        
        read_state(); // update state from sim

        return true;
    } else { 
        return false;
    }
}

void XBotMjSim::read_state() {
    read_dofs();
    read_root();
}

void XBotMjSim::read_dofs() {
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

void XBotMjSim::read_root() {
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

void XBotMjSim::assign_init_root_state() {

    xbot_mujoco::p_init.assign(p_i.begin(),p_i.end());
    xbot_mujoco::q_init.assign(q_i.begin(),q_i.end());
    xbot_mujoco::root_link=base_link_name;

}

bool XBotMjSim::reset() {
    
    if (is_running()) {
        auto& simulation = *xbot_mujoco::sim;
        assign_init_root_state();
        xbot_mujoco::Reset(simulation);
        read_state(); // update state from sim
        return true;
    } else {
        return false;
    }
    
}

void XBotMjSim::clear_sim() {
    xbot_mujoco::ClearSimulation();
    xbot_mujoco::xbot2_wrapper.reset();
}

void XBotMjSim::physics_loop() {
    // InitSimulation has to be called here since it will wait for the render thread to
    // finish loading

    std::unique_lock<std::mutex> lock(ph_step_mtx);

    xbot_mujoco::InitSimulation(xbot_mujoco::sim.get(),model_fname.c_str(),xbot2_config_path.c_str(),
        match_rt_factor // disable automatic rt factor alignment inside mujoco to allow handling it at this level
        );
    reset();

    physics_dt=xbot_mujoco::m->opt.timestep;

    int rt_factor_freq = std::round(rt_factor_dt / physics_dt);
    if (rt_factor_freq <= 0) { // in case physics_dt is bigger than rt_factor_dt
        rt_factor_freq = 1;
    } 
    step_dt_trgt_walltime = rt_factor_freq * physics_dt / rt_factor_trgt; // compute desired walltime sim dt

    if (render_to_file) {
        init_custom_camera();
        custom_camera_ready.store(true);
    }

    initialized.store(true);

    n_dofs = n_jnts();
    jnts_q.resize(n_dofs);
    jnts_v.resize(n_dofs);
    jnts_a.resize(n_dofs);
    jnts_eff.resize(n_dofs);

    dof_names.resize(n_dofs);
    dof_names=jnt_names();

    lock.unlock();

    struct timespec start_time, end_time, sleep_time;
    clock_gettime(CLOCK_MONOTONIC, &start_time);

    while (!((*xbot_mujoco::sim).exitrequest.load())) {
        step_sim();
        steps_counter++;

        if (render_to_file && (steps_counter%render_phstepfreq==0)){
            std::unique_lock<std::mutex> render_lock(render_mtx);
            render_req=true;
            render_done=false;
            render_step_req_cv.notify_all(); // send render request
            if (!render_step_res_cv.wait_for(render_lock, std::chrono::milliseconds(timeout), [this] { return render_done; })) {
                fprintf(stderr,"[xbot2_mujoco][XBotMjSim][physics_loop_manual]: no render acknowledgement received within timeout of %i ms\n", timeout);
                return;
            }
        }
        
        if (match_rt_factor && (steps_counter % rt_factor_freq == 0)) {
            clock_gettime(CLOCK_MONOTONIC, &end_time);

            double elapsed = (end_time.tv_sec - start_time.tv_sec) + (end_time.tv_nsec - start_time.tv_nsec) / 1e9;
            double remaining = step_dt_trgt_walltime - elapsed;

            if (std::abs(remaining) > 1e-6) {
                if (remaining > 0) {
                    sleep_time.tv_sec = static_cast<time_t>(remaining);
                    sleep_time.tv_nsec = static_cast<long>((remaining - sleep_time.tv_sec) * 1e9);
                    clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_time, nullptr);
                } else { // we are lagging
                    // printf("[xbot2_mujoco][XBotMjSim][physics_loop_manual]: requested step walltime cannot be matched (target: %fs VS meas: %fs)!\n", 
                    //     step_dt_trgt_walltime, elapsed);
                }
            }

            clock_gettime(CLOCK_MONOTONIC, &start_time);
        }
    }
    
}

void XBotMjSim::physics_loop_manual() {
    // InitSimulation has to be called here since it will wait for the render thread to finish loading

    std::unique_lock<std::mutex> lock(ph_step_mtx);

    xbot_mujoco::InitSimulation(xbot_mujoco::sim.get(), model_fname.c_str(), xbot2_config_path.c_str(),
        match_rt_factor // disable rt factor alignment within Mujoco to allow handling it at this level
    );

    reset();
    
    physics_dt = xbot_mujoco::m->opt.timestep;
    
    int rt_factor_freq = std::round(rt_factor_dt / physics_dt);
    if (rt_factor_freq <= 0) { // in case physics_dt is bigger than rt_factor_dt
        rt_factor_freq = 1;
    } 
    step_dt_trgt_walltime = rt_factor_freq * physics_dt / rt_factor_trgt; // compute desired walltime sim dt

    if (render_to_file) {
        init_custom_camera();
        custom_camera_ready.store(true);
    }

    initialized.store(true);

    n_dofs = n_jnts();
    jnts_q.resize(n_dofs);
    jnts_v.resize(n_dofs);
    jnts_a.resize(n_dofs);
    jnts_eff.resize(n_dofs);

    dof_names.resize(n_dofs);
    dof_names = jnt_names();

    lock.unlock();
    
    struct timespec start_time, end_time, sleep_time;
    clock_gettime(CLOCK_MONOTONIC, &start_time);

    while (!(xbot_mujoco::sim->exitrequest.load())) {

        std::unique_lock<std::mutex> lock(ph_step_mtx);
        
        if (!sim_step_req_cv.wait_for(lock, std::chrono::milliseconds(timeout), [this] { return step_req; })) {
            printf("[xbot2_mujoco][XBotMjSim][physics_loop_manual]: no step request received within timeout of %i ms\n", timeout);
            xbot_mujoco::sim->exitrequest.store(1);
            return;
        } 

        step_sim();
        steps_counter++;
        step_req = false;
        step_done = true; // signal that simulation has stepped
        
        if (render_to_file && (steps_counter%render_phstepfreq==0)){
            std::unique_lock<std::mutex> render_lock(render_mtx);
            render_req=true;
            render_done=false;
            render_step_req_cv.notify_all(); // send render request
            if (!render_step_res_cv.wait_for(render_lock, std::chrono::milliseconds(timeout), [this] { return render_done; })) {
                fprintf(stderr,"[xbot2_mujoco][XBotMjSim][physics_loop_manual]: no render acknowledgement received within timeout of %i ms\n", timeout);
                return;
            }
        }

        sim_step_res_cv.notify_all();

        if (match_rt_factor && (steps_counter % rt_factor_freq == 0)) {
            clock_gettime(CLOCK_MONOTONIC, &end_time);

            double elapsed = (end_time.tv_sec - start_time.tv_sec) + (end_time.tv_nsec - start_time.tv_nsec) / 1e9;
            double remaining = step_dt_trgt_walltime - elapsed;

            if (std::abs(remaining) > 1e-6) {
                if (remaining > 0) {
                    sleep_time.tv_sec = static_cast<time_t>(remaining);
                    sleep_time.tv_nsec = static_cast<long>((remaining - sleep_time.tv_sec) * 1e9);
                    clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_time, nullptr);
                } else { // we are lagging
                    // printf("[xbot2_mujoco][XBotMjSim][physics_loop_manual]: requested step walltime cannot be matched (target: %fs VS meas: %fs)!\n", 
                    //     step_dt_trgt_walltime, elapsed);
                }
            }

            clock_gettime(CLOCK_MONOTONIC, &start_time);
        }
    }
}

void XBotMjSim::rendering_loop() {
    
    xbot_mujoco::sim->RenderLoopSetup();

    xbot_mujoco::sim->frames_ = 0;
    xbot_mujoco::sim->last_fps_update_ = mj::Simulate::Clock::now();

    // run event loop
    while (!xbot_mujoco::sim->platform_ui->ShouldCloseWindow() && !xbot_mujoco::sim->exitrequest.load()) {
    {
        const mj::MutexLock lock(xbot_mujoco::sim->mtx);

        // load model (not on first pass, to show "loading" label)
        if (xbot_mujoco::sim->loadrequest==1) {
        xbot_mujoco::sim->LoadOnRenderThread();
        } else if (xbot_mujoco::sim->loadrequest == 2) {
        xbot_mujoco::sim->loadrequest = 1;
        }

        // poll and handle events
        if (!headless) {
        xbot_mujoco::sim->platform_ui->PollEvents();
        }

        // upload assets if requested
        bool upload_notify = false;
        if (xbot_mujoco::sim->hfield_upload_ != -1) {
        mjr_uploadHField(xbot_mujoco::m, &xbot_mujoco::sim->platform_ui->mjr_context(), xbot_mujoco::sim->hfield_upload_);
        xbot_mujoco::sim->hfield_upload_ = -1;
        upload_notify = true;
        }
        if (xbot_mujoco::sim->mesh_upload_ != -1) {
        mjr_uploadMesh(xbot_mujoco::m, &xbot_mujoco::sim->platform_ui->mjr_context(), xbot_mujoco::sim->mesh_upload_);
        xbot_mujoco::sim->mesh_upload_ = -1;
        upload_notify = true;
        }
        if (xbot_mujoco::sim->texture_upload_ != -1) {
        mjr_uploadTexture(xbot_mujoco::m, &xbot_mujoco::sim->platform_ui->mjr_context(), xbot_mujoco::sim->texture_upload_);
        xbot_mujoco::sim->texture_upload_ = -1;
        upload_notify = true;
        }
        if (upload_notify) {
        xbot_mujoco::sim->cond_upload_.notify_all();
        }

        // update scene, doing a full sync if in fully managed mode
        if (!xbot_mujoco::sim->is_passive_) {
        xbot_mujoco::sim->Sync();
        } else {
        xbot_mujoco::sim->scnstate_.data.warning[mjWARN_VGEOMFULL].number += mjv_updateSceneFromState(
            &xbot_mujoco::sim->scnstate_, &xbot_mujoco::sim->opt, &xbot_mujoco::sim->pert, &xbot_mujoco::sim->cam, mjCAT_ALL, &xbot_mujoco::sim->scn);
        }

    }  // MutexLock (unblocks simulation thread)

    // render while simulation is running
    if (!headless) {
        
        if (render_to_file && custom_camera_ready.load()) {
            // synchronous rendering (triggered by physycs loop at contant freq)
            std::unique_lock<std::mutex> render_lock(render_mtx);
            if (!render_step_req_cv.wait_for(render_lock, std::chrono::milliseconds(timeout), [this] { return render_req; })) {
                printf("[xbot2_mujoco][XBotMjSim][rendering_loop]: no step request received within timeout of %i ms\n", timeout);
                return;
            }
            xbot_mujoco::sim->Render();
            render_png(steps_counter);
            render_req=false;
            render_done=true; 
            render_step_res_cv.notify_all();
    
        } else { // asynch rendering
            xbot_mujoco::sim->Render();
        }
        // update FPS stat, at most 5 times per second
        auto now = mj::Simulate::Clock::now();
        double interval = std::chrono::duration<double>(now - xbot_mujoco::sim->last_fps_update_).count();
        ++xbot_mujoco::sim->frames_;
        if (interval > 0.2) {
        xbot_mujoco::sim->last_fps_update_ = now;
        xbot_mujoco::sim->fps_ = xbot_mujoco::sim->frames_ / interval;
        xbot_mujoco::sim->frames_ = 0;
        }
    } else { // avoid busy loop
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
    }

    const mj::MutexLock lock(xbot_mujoco::sim->mtx);
    mjv_freeScene(&xbot_mujoco::sim->scn);
    if (xbot_mujoco::sim->is_passive_) {
    mjv_freeSceneState(&xbot_mujoco::sim->scnstate_);
    }

    xbot_mujoco::sim->exitrequest.store(2);
}

void XBotMjSim::step_sim() {

    xbot_mujoco::PreStep(*xbot_mujoco::sim);
    xbot_mujoco::DoStep(*xbot_mujoco::sim,syncCPU,syncSim);
    step_counter=xbot_mujoco::step_counter;

}

void XBotMjSim::initialize(bool headless) {
    
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
    std::printf("[XBotMjSim][initialize]: MuJoCo version %s\n", mj_versionString());
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
        &cam, &opt, &pert, /* is_passive = */ false,
        headless
    );

    mjcb_control = xbot_mujoco::xbotmj_control_callback; // register control callback to mujoco
    
    assign_init_root_state();    

    // Install the signal handler for SIGINT (Ctrl+C)
    std::signal(SIGINT, handle_sigint);
    // physics_thread = std::thread(&xbot_mujoco::SimulationLoop, xbot_mujoco::sim.get(), 
    //     model_fname.c_str(), xbot2_config_path.c_str());
    if (!manual_stepping) {
        printf("[xbot2_mujoco][XBotMjSim][initialize]: launching physics sim thread\n");
        physics_thread = std::thread(&XBotMjSim::physics_loop, this);
    } else { // will (atomically) wait for step to be called
        printf("[xbot2_mujoco][XBotMjSim][initialize]: launching physics sim thread with manual stepping\n");
        physics_thread = std::thread(&XBotMjSim::physics_loop_manual, this);
    }

    rendering_loop(); // render in this thread 
    
    // (if headlees no actual rendering is performed)
    if (physics_thread.joinable()) {
        physics_thread.join();
        printf("[xbot2_mujoco][XBotMjSim][close]: physics thread terminated\n");
    }

    clear_sim();

}

void XBotMjSim::init_custom_camera(){
    int cam_id = mj_name2id(xbot_mujoco::m, mjOBJ_CAMERA, custom_camera_name.c_str());

    // use custom camera options
    mjv_defaultCamera(&custom_mj_cam); 

    // depth = new float[custom_cam_width * custom_cam_height];
    custom_cam_rect= {0, 0, custom_cam_width, custom_cam_height};
    float physics_fps = 1.0/physics_dt;
    render_phstepfreq = std::round(physics_fps/render_fps);

    std::string unique_id = get_current_time_as_string();
    
    std::filesystem::create_directory(render_base_path+"/XBotMjSimRenderings");
    std::filesystem::create_directory(render_base_path+"/XBotMjSimRenderings/"+ unique_id);

    render_path=render_base_path+"/XBotMjSimRenderings/"+ unique_id+"/"+custom_camera_name;

}

void XBotMjSim::render_png(int frame_idx){
    
    if (&xbot_mujoco::sim->platform_ui->mjr_context() && &xbot_mujoco::sim->scn) {
        // Update the scene using the offscreen camera
        // mjv_updateScene(xbot_mujoco::m, xbot_mujoco::d, 
        //     &xbot_mujoco::sim->opt,
        //     &xbot_mujoco::sim->pert,
        //     &custom_mj_cam, 
        //     mjCAT_ALL, 
        //     &xbot_mujoco::sim->scn);

        // Render to the offscreen buffer
        // mjr_setBuffer(mjFB_OFFSCREEN, &xbot_mujoco::sim->platform_ui->mjr_context());

        // mjr_render(custom_cam_rect, &xbot_mujoco::sim->scn, &xbot_mujoco::sim->platform_ui->mjr_context());

        static std::unique_ptr<unsigned char[]> rgb_data(new unsigned char[3*custom_cam_width*custom_cam_height]);

        mjr_readPixels(rgb_data.get(), nullptr, xbot_mujoco::sim->uistate.rect[0], &xbot_mujoco::sim->platform_ui->mjr_context());
        // flip up-down
        for (int r = 0; r < custom_cam_height/2; ++r) {
            unsigned char* top_row = &rgb_data[3*custom_cam_width*r];
            unsigned char* bottom_row = &rgb_data[3*custom_cam_width*(custom_cam_height-1-r)];
            std::swap_ranges(top_row, top_row+3*custom_cam_width, bottom_row);
        }

        std::string render_path_now=render_path+std::to_string(frame_idx) + ".png"; 

        // save as PNG
        // TODO(b/241577466): Parse the stem of the filename and use a .PNG extension.
        // Unfortunately, if we just yank ".xml"/".mjb" from the filename and append .PNG, the macOS
        // file dialog does not automatically open that location. Thus, we defer to a default
        // "screenshot.png" for now.

        if (!render_path_now.empty()) {
        if (lodepng::encode(render_path_now, rgb_data.get(), custom_cam_width, custom_cam_height, LCT_RGB)) {
            mju_error("could not save screenshot");
        }
        }
    }
}

bool XBotMjSim::is_running() {
    return running.load();
}

bool XBotMjSim::run() {

    if (!is_running()) {

        // std::vector<std::string> args;
        // std::vector<char*> argv;
        // for (std::string& arg : args)
        // {
        //     argv.push_back(&arg[0]);
        // }
        // int argc = argv.size();
        // ros::init(argc, argv.data(), std::string("XBotMjSim"));

        simulator_thread = std::thread(&XBotMjSim::initialize, this, headless);

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

std::vector<std::string> XBotMjSim::jnt_names() {
    return xbot_mujoco::JntNames(xbot_mujoco::m);
}

int XBotMjSim::n_jnts() {
    std::vector<std::string> names=xbot_mujoco::JntNames(xbot_mujoco::m);
    return names.size();
}

void XBotMjSim::move_to_homing_now() {
    xbot_mujoco::MoveJntsToHomingNow(xbot_mujoco::d);
}

void XBotMjSim::move_base_to_now(std::vector<double> p, std::vector<double> q) {
    xbot_mujoco::MoveBaseNowTo(xbot_mujoco::d, p, q, base_link_name);
}