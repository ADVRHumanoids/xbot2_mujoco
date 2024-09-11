// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "simulator.h"
#include <csignal>  // For signal handling

// initializing extern variables
mjModel* xbot_mujoco::m = NULL;
mjData* xbot_mujoco::d = NULL;
// control noise variables
mjtNum* xbot_mujoco::ctrlnoise = nullptr;
int xbot_mujoco::step_counter = 0;
std::unique_ptr<xbot_mujoco::mj::Simulate> xbot_mujoco::sim;
// utility objs
std::tuple<std::vector<std::string>, std::vector<double>> xbot_mujoco::homing;
std::vector<double> xbot_mujoco::p_init = {0.0, 0.0, 0.8}; 
std::vector<double> xbot_mujoco::q_init = {1.0, 0.0, 0.0, 0.0};
std::string xbot_mujoco::root_link="root_link";

//xbot2
XBot::MjWrapper::UniquePtr xbot_mujoco::xbot2_wrapper;

void xbot_mujoco::handle_sigint(int signal_num) {
  std::printf("[xbot2_mujoco][simulator]: detected SIGINT -> exiting gracefully \n");
  xbot_mujoco::sim->exitrequest.store(1); // signal sim ad rendering loop to exit gracefully
}

//---------------------------------------- plugin handling -----------------------------------------

// return the path to the directory containing the current executable
// used to determine the location of auto-loaded plugin libraries
std::string xbot_mujoco::getExecutableDir() {
#if defined(_WIN32) || defined(__CYGWIN__)
  constexpr char kPathSep = '\\';
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    DWORD buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new(std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);
      if (written < buf_size) {
        success = true;
      } else if (written == buf_size) {
        // realpath is too small, grow and retry
        buf_size *=2;
      } else {
        std::cerr << "failed to retrieve executable path: " << GetLastError() << "\n";
        return "";
      }
    }
    return realpath.get();
  }();
#else
  constexpr char kPathSep = '/';
#if defined(__APPLE__)
  std::unique_ptr<char[]> buf(nullptr);
  {
    std::uint32_t buf_size = 0;
    _NSGetExecutablePath(nullptr, &buf_size);
    buf.reset(new char[buf_size]);
    if (!buf) {
      std::cerr << "cannot allocate memory to store executable path\n";
      return "";
    }
    if (_NSGetExecutablePath(buf.get(), &buf_size)) {
      std::cerr << "unexpected error from _NSGetExecutablePath\n";
    }
  }
  const char* path = buf.get();
#else
  const char* path = "/proc/self/exe";
#endif
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    std::uint32_t buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new(std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      std::size_t written = readlink(path, realpath.get(), buf_size);
      if (written < buf_size) {
        realpath.get()[written] = '\0';
        success = true;
      } else if (written == -1) {
        if (errno == EINVAL) {
          // path is already not a symlink, just use it
          return path;
        }

        std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
        return "";
      } else {
        // realpath is too small, grow and retry
        buf_size *= 2;
      }
    }
    return realpath.get();
  }();
#endif

  if (realpath.empty()) {
    return "";
  }

  for (std::size_t i = realpath.size() - 1; i > 0; --i) {
    if (realpath.c_str()[i] == kPathSep) {
      return realpath.substr(0, i);
    }
  }

  // don't scan through the entire file system's root
  return "";
}

// scan for libraries in the plugin directory to load additional plugins
void xbot_mujoco::scanPluginLibraries() {
  // check and print plugins that are linked directly into the executable
  int nplugin = mjp_pluginCount();
  if (nplugin) {
    std::printf("Built-in plugins:\n");
    for (int i = 0; i < nplugin; ++i) {
      std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
  }

  // define platform-specific strings
#if defined(_WIN32) || defined(__CYGWIN__)
  const std::string sep = "\\";
#else
  const std::string sep = "/";
#endif


  // try to open the ${EXECDIR}/MUJOCO_PLUGIN_DIR directory
  // ${EXECDIR} is the directory containing the simulate binary itself
  // MUJOCO_PLUGIN_DIR is the MUJOCO_PLUGIN_DIR preprocessor macro
  const std::string executable_dir = xbot_mujoco::getExecutableDir();
  if (executable_dir.empty()) {
    return;
  }

  const std::string plugin_dir = xbot_mujoco::getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
  mj_loadAllPluginLibraries(
      plugin_dir.c_str(), +[](const char* filename, int first, int count) {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
      });
}

//--------------------------- rendering and simulation ----------------------------------
std::vector<std::string> JntNames(mjModel* m) {

  std::vector<std::string> jnt_names;

  for(int i = 0; i < m->njnt; i++)
  {
    std::string jname = &m->names[m->name_jntadr[i]];

    if(m->jnt_type[i] != mjtJoint::mjJNT_HINGE &&
            m->jnt_type[i] != mjtJoint::mjJNT_SLIDE)
    { // not supported
      fprintf(stderr, "[xbot2_mujoco][JntNames]: Joint %s neither of type mjJNT_HINGE nor mjJNT_SLIDE. Will be ignored", 
        jname.c_str());
      continue;
    } else {
      jnt_names.push_back(jname);
    }
  }

  return jnt_names;
}

void xbot_mujoco::SetJntOffsets(mjModel* m) {
  
  for(int i = 0; i < std::get<0>(homing).size(); i++)
  {

    int joint_id = mj_name2id(m, mjOBJ_JOINT, std::get<0>(homing)[i].c_str());
    if (joint_id != -1) {// joint found -> set default joint pos
        int qpos_adr = m->jnt_qposadr[joint_id];
        m->qpos0[qpos_adr] = std::get<1>(homing)[i];
    }
  }  

}

void xbot_mujoco::MoveJntsToHomingNow(mjData* d) {

  for(int i = 0; i < std::get<0>(homing).size(); i++)
  {
    int joint_id = mj_name2id(m, mjOBJ_JOINT, std::get<0>(homing)[i].c_str());

    if (joint_id != -1) {// joint found -> set joint pos
        int qpos_adr = m->jnt_qposadr[joint_id];
        d->qpos[qpos_adr] = std::get<1>(homing)[i];
        // d->qvel[qpos_adr] = 0.0;
        // d->ctrl[qpos_adr] = 0.0;
    }
  }
}

void xbot_mujoco::MoveBaseNowTo(mjData* d, std::vector<double> p, std::vector<double> q,
  std::string root_linkname) {

  // Number of bodies in the model

  int rott_link_idx = mj_name2id(m, mjOBJ_BODY, root_linkname.c_str());

  if (rott_link_idx != -1) { // root link found

    int floating_jnt_address = m->jnt_qposadr[m->body_jntadr[rott_link_idx]];

    d->qpos[floating_jnt_address+0] = p[0];
    d->qpos[floating_jnt_address+1] = p[1];
    d->qpos[floating_jnt_address+2] = p[2];

    d->qpos[floating_jnt_address+3] = q[0];
    d->qpos[floating_jnt_address+4] = q[1];
    d->qpos[floating_jnt_address+5] = q[2];
    d->qpos[floating_jnt_address+6] = q[3];
  }
}

mjModel* xbot_mujoco::LoadModel(const char* file, mj::Simulate& sim) {
  // this copy is needed so that the mju::strlen call below compiles
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  // make sure filename is not empty
  if (!filename[0]) {
    return nullptr;
  }

  // load and compile
  char loadError[kErrorLength] = "";
  mjModel* mnew = 0;
  if (mju::strlen_arr(filename)>4 &&
      !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                    mju::sizeof_arr(filename) - mju::strlen_arr(filename)+4)) {
    printf( "[xbot2_mujoco][simulator][LoadModel]: trying to load binary model at %s \n",filename);
    mnew = mj_loadModel(filename, nullptr);
    if (!mnew) {
      mju::strcpy_arr(loadError, "could not load binary model");
    }

  } else {
    printf( "[xbot2_mujoco][simulator][LoadModel]: trying to load XML model at %s \n",filename);

    mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);
    // remove trailing newline character from loadError

    if (loadError[0]) {
      int error_length = mju::strlen_arr(loadError);
      if (loadError[error_length-1] == '\n') {
        loadError[error_length-1] = '\0';
      }
    }
  }

  mju::strcpy_arr(sim.load_error, loadError);

  if (!mnew) {
    std::printf("%s\n", loadError);
    return nullptr;
  }

  // compiler warning: print and pause
  if (loadError[0]) {
    // mj_forward() below will print the warning message
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
    sim.run = 0;
  }

  return mnew;
}

void xbot_mujoco::DoStep(mj::Simulate& sim, 
    std::chrono::time_point<mj::Simulate::Clock> syncCPU,
    mjtNum syncSim) {
    // lock the sim mutex
    const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

    // run only if model is present
    if (m && d) {
        // running
        if (sim.run) {

            if (sim.resetrequest.load()) { // perform reset
              step_counter=0;
              sim.resetrequest.store(0); // reset performed
            }

            if (step_counter==0) {
              xbot_mujoco::Reset(sim);
            }

            bool stepped = false;

            // record cpu time at start of iteration
            const auto startCPU = mj::Simulate::Clock::now();

            // elapsed CPU and simulation time since last sync
            const auto elapsedCPU = startCPU - syncCPU;
            double elapsedSim = d->time - syncSim;

            // inject noise
            if (sim.ctrl_noise_std) {
                // convert rate and scale to discrete time (Ornstein–Uhlenbeck)
                mjtNum rate = mju_exp(-m->opt.timestep / mju_max(sim.ctrl_noise_rate, mjMINVAL));
                mjtNum scale = sim.ctrl_noise_std * mju_sqrt(1-rate*rate);

                for (int i=0; i<m->nu; i++) {
                // update noise
                ctrlnoise[i] = rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);

                // apply noise
                d->ctrl[i] = ctrlnoise[i];
                }
            }

            // requested slow-down factor
            double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

            // misalignment condition: distance from target sim time is bigger than syncmisalign
            bool misaligned =
                mju_abs(Seconds(elapsedCPU).count()/slowdown - elapsedSim) > syncMisalign;

            // out-of-sync (for any reason): reset sync times, step
            if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
                misaligned || sim.speed_changed) {
                // re-sync
                syncCPU = startCPU;
                syncSim = d->time;
                sim.speed_changed = false;

                // run single step, let next iteration deal with timing
                mj_step(m, d);
                stepped = true;
            }

            // in-sync: step until ahead of cpu
            else {
                bool measured = false;
                mjtNum prevSim = d->time;

                double refreshTime = simRefreshFraction/sim.refresh_rate;

                // step while sim lags behind cpu and within refreshTime
                while (Seconds((d->time - syncSim)*slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                        mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime)) {
                    // measure slowdown before first step
                    if (!measured && elapsedSim) {
                    sim.measured_slowdown =
                        std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                    measured = true;
                    }

                    // call mj_step
                    mj_step(m, d);
                    stepped = true;

                    // break if reset
                    if (d->time < prevSim) {
                    break;
                    }
                }
            }

            // save current state to history buffer
            if (stepped) {
            sim.AddToHistory();
            }
          
            step_counter++;

        }

        // paused
        else {
          // run mj_forward, to update rendering and joint sliders
          mj_forward(m, d);
          sim.speed_changed = true;
        }
    } else {
        printf("[xbot2_mujoco][simulator][DoStep]: either m or d are NULL!!\n");
    }
}

void xbot_mujoco::PreStep(mj::Simulate& sim) {
  
  if (sim.droploadrequest.load()) {
      sim.LoadMessage(sim.dropfilename);
      mjModel* mnew = xbot_mujoco::LoadModel(sim.dropfilename, sim);
      sim.droploadrequest.store(false);

      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.dropfilename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);

        // allocate ctrlnoise
        free(ctrlnoise);
        ctrlnoise = (mjtNum*) malloc(sizeof(mjtNum)*m->nu);
        mju_zero(ctrlnoise, m->nu);
      } else {
        sim.LoadMessageClear();
      }
    }

    if (sim.uiloadrequest.load()) {
      sim.uiloadrequest.fetch_sub(1);
      sim.LoadMessage(sim.filename);
      mjModel* mnew = xbot_mujoco::LoadModel(sim.filename, sim);
      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.filename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);

        // allocate ctrlnoise
        free(ctrlnoise);
        ctrlnoise = static_cast<mjtNum*>(malloc(sizeof(mjtNum)*m->nu));
        mju_zero(ctrlnoise, m->nu);
      } else {
        sim.LoadMessageClear();
      }
    }

    // sleep for 1 ms or yield, to let main thread run
    //  yield results in busy wait - which has better timing but kills battery life
    if (sim.run && sim.busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

// simulate in background thread (while rendering in main thread)
void xbot_mujoco::PhysicsLoop(mj::Simulate& sim) {
  // cpu-sim syncronization point
  std::chrono::time_point<mj::Simulate::Clock> syncCPU;
  mjtNum syncSim = 0;

  // run until asked to exit
  while (!sim.exitrequest.load()) {
    
    PreStep(sim);
    
    DoStep(sim,syncCPU,syncSim);

  }
}

void xbot_mujoco::InitSimulation(mj::Simulate* sim, const char* mj_filename, const char* xbot_config_path) {
  // request loadmodel if file given (otherwise drag-and-drop)
  if (mj_filename != nullptr) {
    sim->LoadMessage(mj_filename);

    m = xbot_mujoco::LoadModel(mj_filename, *sim);  

    if (m) {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      d = mj_makeData(m);
    }
    if (d) {
      sim->Load(m, d, mj_filename);
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      mj_forward(m, d);

      // allocate ctrlnoise
      free(ctrlnoise);
      ctrlnoise = static_cast<mjtNum*>(malloc(sizeof(mjtNum)*m->nu));
      mju_zero(ctrlnoise, m->nu);
    } else {
      sim->LoadMessageClear();
    }
  }

  homing = LoadingUtils::generate_ordered_homing(xbot_config_path);

  LoadingUtils::print_homing(std::get<0>(homing), std::get<1>(homing));

  xbot2_wrapper.reset();

  xbot2_wrapper = std::make_unique<XBot::MjWrapper>(m, xbot_config_path);

}

void xbot_mujoco::ClearSimulation() {
  // delete everything we allocated
  free(ctrlnoise);
  mj_deleteData(d);
  mj_deleteModel(m);

  std::printf("[xbot2_mujoco][simulator]: simulation loop terminated. \n");
}

void xbot_mujoco::SimulationLoop(mj::Simulate* sim, const char* mj_filename, const char* xbot_config_path) {
  
  xbot_mujoco::InitSimulation(sim,mj_filename,xbot_config_path);

  xbot_mujoco::PhysicsLoop(*sim);

  xbot_mujoco::ClearSimulation();
}

void xbot_mujoco::RenderingLoop(mj::Simulate* sim, ros::NodeHandle nh) {
  sim->RenderLoop(nh);
  std::printf("[xbot2_mujoco][simulator]: rendering loop terminated. \n");
}

void xbot_mujoco::Reset(mj::Simulate& sim) {

  xbot_mujoco::MoveJntsToHomingNow(d);
  xbot_mujoco::MoveBaseNowTo(d,p_init,q_init,root_link);
  xbot2_wrapper->reset(d);
  step_counter==0;
}

void xbot_mujoco::run(const char* fname, 
    const std::string xbot2_config_path,
    ros::NodeHandle nh,
    bool headless)
{
  // Install the signal handler for SIGINT (Ctrl+C)
  std::signal(SIGINT, xbot_mujoco::handle_sigint);

  p_init[0] = 0.0;
  p_init[1] = 0.0;
  p_init[2] = 1.0;

  q_init[0] = 1.0; // MuJoCo uses the (w, x, y, z) convention for quaternions
  q_init[1] = 0.0;
  q_init[2] = 0.0;
  q_init[3] = 0.0;

  root_link="base_link";

  // display an error if running on macOS under Rosetta 2
  #if defined(__APPLE__) && defined(__AVX__)
  if (rosetta_error_msg) {
      DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
      std::exit(1);
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
  sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &cam, &opt, &pert, /* is_passive = */ false
  );
  
  mjcb_control = xbot_mujoco::xbotmj_control_callback; // register control callback to mujoco
  
  // start physics thread
  std::thread physicsthreadhandle(&SimulationLoop, sim.get(), fname, xbot2_config_path.c_str());
  
  RenderingLoop(sim.get(), nh); // render in this thread

  if (physicsthreadhandle.joinable()) {
    physicsthreadhandle.join();
  }

  xbot2_wrapper.reset();
}

void xbot_mujoco::reset(mj::Simulate& sim) {
  sim.resetrequest.load();
}

//-------------------------------- control callback ----------------------------------------

void xbot_mujoco::xbotmj_control_callback(const mjModel* m, mjData* d)
{

    if(!xbot2_wrapper)
    {
        return;
    }

    xbot2_wrapper->run(d);

}


