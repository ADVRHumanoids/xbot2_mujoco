#include "mj_sim_env.h"

MjSimEnv::MjSimEnv(const char* configPath) : xbot2_cfg_path(configPath) {
    
    window = glfwCreateWindow(800, 600, "MuJoCo Simulation", NULL, NULL);
    if (!window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwSetDropCallback(window, dropCallback);
    glfwSetKeyCallback(window, keyCallback);

    // Initialize MuJoCo and other members
    // ...

    loadModel(); // Load the initial model
}

MjSimEnv::~MjSimEnv() {
    if (simThread.joinable()) {
        settings.exitrequest = 1;
        simThread.join();
    }

    fprintf(stderr, "destroying xbot2 wrapper \n");
    xbot2_wrapper.reset();

    // delete everything we allocated
    uiClearCallback(window);
    mj_deleteData(d);
    mj_deleteModel(m);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif
}

void MjSimEnv::init() {
    // print version, check compatibility
    printf("MuJoCo Pro version %.2lf\n", 0.01*mj_version());
    if( mjVERSION_HEADER!=mj_version() )
        mju_error("Headers and library have different versions");
    // init GLFW, set timer callback (milliseconds)
    if (!glfwInit())
        mju_error("could not initialize GLFW");
    mjcb_time = timer;

    // multisampling
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_VISIBLE, 1);

    // get videomode and save
    vmode = *glfwGetVideoMode(glfwGetPrimaryMonitor());

    window = glfwCreateWindow((2*vmode.width)/3, (2*vmode.height)/3,
                              "Simulate", NULL, NULL);
    if( !window )
    {
        glfwTerminate();
        mju_error("could not create window");
    }

    // save window position and size
    glfwGetWindowPos(window, windowpos, windowpos+1);
    glfwGetWindowSize(window, windowsize, windowsize+1);

    // make context current, set v-sync
    glfwMakeContextCurrent(window);
    glfwSwapInterval(settings.vsync);

    // init abstract visualization
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&vopt);
    profilerinit();
    sensorinit();

    // make empty scene
    mjv_defaultScene(&scn);
    mjv_makeScene(NULL, &scn, maxgeom);

    // select default font
    int fontscale = uiFontScale(window);
    settings.font = fontscale/50 - 1;

    // make empty context
    mjr_defaultContext(&con);
    mjr_makeContext(NULL, &con, fontscale);

    // set GLFW callbacks
    uiSetCallback(window, &uistate, uiEvent, uiLayout);
    glfwSetWindowRefreshCallback(window, render);
    glfwSetDropCallback(window, drop);

    // init state and uis
    memset(&uistate, 0, sizeof(mjuiState));
    memset(&ui0, 0, sizeof(mjUI));
    memset(&ui1, 0, sizeof(mjUI));
    ui0.spacing = mjui_themeSpacing(settings.spacing);
    ui0.color = mjui_themeColor(settings.color);
    ui0.predicate = uiPredicate;
    ui0.rectid = 1;
    ui0.auxid = 0;
    ui1.spacing = mjui_themeSpacing(settings.spacing);
    ui1.color = mjui_themeColor(settings.color);
    ui1.predicate = uiPredicate;
    ui1.rectid = 2;
    ui1.auxid = 1;

    // populate uis with standard sections
    mjui_add(&ui0, defFile);
    mjui_add(&ui0, defOption);
    mjui_add(&ui0, defSimulation);
    mjui_add(&ui0, defWatch);
    uiModify(window, &ui0, &uistate, &con);
    uiModify(window, &ui1, &uistate, &con);
}

void MjSimEnv::run() {
    running = true;
    simThread = std::thread(&MjSimEnv::simulate, this);

    // event loop
    while( !glfwWindowShouldClose(window) && !settings.exitrequest )
    {
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

    running = false;
}

void MjSimEnv::handleEvent(const mjuiState* state) {
    uiEvent(state);
}

void MjSimEnv::updateUI() {
    // Update UI elements
    // ...
}

void MjSimEnv::prepare() {
    lastUpdateTime = 0;
    double currentTime = glfwGetTime();
    double interval = currentTime - lastUpdateTime;
    interval = mjMIN(1, mjMAX(0.0001, interval));
    lastupdatetm = currentTime;

    // no model: nothing to do
    if( !m )
        return;

    // update scene
    mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scn);

    // update watch
    if( settings.ui0 && ui0.sect[SECT_WATCH].state )
    {
        watch();
        mjui_update(SECT_WATCH, -1, &ui0, &uistate, &con);
    }

    // ipdate joint
    if( settings.ui1 && ui1.sect[SECT_JOINT].state )
            mjui_update(SECT_JOINT, -1, &ui1, &uistate, &con);

    // update info text
    if( settings.info )
        infotext(info_title, info_content, interval);

    // update profiler
    if( settings.profiler && settings.run )
        profilerupdate();

    // update sensor
    if( settings.sensor && settings.run )
        sensorupdate();

    // clear timers once profiler info has been copied
    cleartimers();
}

void MjSimEnv::render(GLFWwindow* window) {
    mjrRender(uistate.rect[3], &scn, &con);
    glfwSwapBuffers(window);
}

void MjSimEnv::simulate() {
    // cpu-sim syncronization point
    double cpusync = 0;
    mjtNum simsync = 0;

    // run until asked to exit
    while(!settings.exitrequest)
    {
        // sleep for 1 ms or yield, to let main thread run
        //  yield results in busy wait - which has better timing but kills battery life
        if(settings.run && settings.busywait)
            std::this_thread::yield();
        else
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // start exclusive access
        mtx.lock();

        // run only if model is present
        if(m)
        {
            // record start time
            double startwalltm = glfwGetTime();

            // running
            if(settings.run)
            {
                // record cpu time at start of iteration
                double tmstart = glfwGetTime();

                // out-of-sync (for any reason)
                if( d->time<simsync || tmstart<cpusync || cpusync==0 ||
                    mju_abs((d->time-simsync)-(tmstart-cpusync))>syncmisalign )
                {
                    // re-sync
                    cpusync = tmstart;
                    simsync = d->time;

                    // clear old perturbations, apply new
                    mju_zero(d->xfrc_applied, 6*m->nbody);
                    mjv_applyPerturbPose(m, d, &pert, 0);  // move mocap bodies only
                    mjv_applyPerturbForce(m, d, &pert);

                    // run single step, let next iteration deal with timing
                    mj_step(m, d);
                }

                // in-sync
                else
                {
                    // step while simtime lags behind cputime, and within safefactor
                    while( (d->time-simsync)<(glfwGetTime()-cpusync) &&
                           (glfwGetTime()-tmstart)<refreshfactor/vmode.refreshRate )
                    {
                        // clear old perturbations, apply new
                        mju_zero(d->xfrc_applied, 6*m->nbody);
                        mjv_applyPerturbPose(m, d, &pert, 0);  // move mocap bodies only
                        mjv_applyPerturbForce(m, d, &pert);

                        // run mj_step
                        mjtNum prevtm = d->time;
                        mj_step(m, d);

                        // break on reset
                        if( d->time<prevtm )
                            break;
                    }
                }
            }

            // paused
            else
            {
                // apply pose perturbation
                mjv_applyPerturbPose(m, d, &pert, 1);      // move mocap and dynamic bodies

                // run mj_forward, to update rendering and joint sliders
                mj_forward(m, d);
            }
        }

        // end exclusive access
        mtx.unlock();
    }
}

void MjSimEnv::alignScale() {
    cam.lookat[0] = m->stat.center[0];
    cam.lookat[1] = m->stat.center[1];
    cam.lookat[2] = m->stat.center[2];
    cam.distance = 1.5 * m->stat.extent;
    cam.type = mjCAMERA_FREE;
}

void MjSimEnv::copyKey() {
    // Copy qpos to clipboard as key
    // ...
}

mjtNum MjSimEnv::timer() {
    return (mjtNum)(1000 * glfwGetTime());
}

void MjSimEnv::clearTimers() {
    for (int i = 0; i < mjNTIMER; i++) {
        d->timer[i].duration = 0;
        d->timer[i].number = 0;
    }
}

void MjSimEnv::updateSettings() {
    // Update settings based on current state
    // ...
}

void MjSimEnv::dropFile(GLFWwindow* window, int count, const char** paths) {
    if (count > 0) {
        mju_strncpy(filename, paths[0], 1000);
        settings.loadrequest = 1;
    }
}

void MjSimEnv::loadModel() {
    // Load the model and update related components
    // ...
}

void MjSimEnv::dropCallback(GLFWwindow* window, int count, const char** paths) {
    MjSimEnv* env = static_cast<MjSimEnv*>(glfwGetWindowUserPointer(window));
    env->dropFile(window, count, paths);
}

void MjSimEnv::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS) {
        MjSimEnv* env = static_cast<MjSimEnv*>(glfwGetWindowUserPointer(window));
        // Handle key events
        // ...
    }
}