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

#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include "glfw_adapter.h"
#include "simulate.h"
#include "array_safety.h"
#include "xbot2_bridge.h"

#include <ros/ros.h> // used for publishing sim time on ros clock

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

extern "C" {
#if defined(_WIN32) || defined(__CYGWIN__)
  #include <windows.h>
#else
  #if defined(__APPLE__)
    #include <mach-o/dyld.h>
  #endif
  #include <sys/errno.h>
  #include <unistd.h>
#endif
}

namespace xbot_mujoco{
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

// model and data
static mjModel* m = NULL;
static mjData* d = NULL;
// control noise variables
static mjtNum* ctrlnoise = nullptr;

using Seconds = std::chrono::duration<double>;

// constants
const double syncMisalign = 0.1;        // maximum mis-alignment before re-sync (simulation seconds)
const double simRefreshFraction = 0.7;  // fraction of refresh available for simulation
const int kErrorLength = 1024;          // load error string length

static int step_counter = 0;

static std::unique_ptr<mj::Simulate> sim;
// xbot2
static XBot::MjWrapper::UniquePtr xbot2_wrapper;

static void handle_sigint(int signal_num) {
  std::printf("[xbot2_mujoco][simulator]: detected SIGINT -> exiting gracefully \n");
  xbot_mujoco::sim->exitrequest.store(1); // signal sim ad rendering loop to exit gracefully
}

// utility objs
static std::tuple<std::vector<std::string>, std::vector<double>> homing;
static std::vector<double> p_init(3, 0.0); 
static std::vector<double> q_init(4, 0.0);
static std::string root_link="root_link";
void xbotmj_control_callback(const mjModel* m, mjData* d);

//---------------------------------------- plugin handling -----------------------------------------

std::string getExecutableDir();
void scanPluginLibraries();

//--------------------------- rendering and simulation ----------------------------------

void MoveJntToHomingNow(mjData* d);
void MoveBaseNowTo(mjData* d, std::vector<double> p, std::vector<double> q,
  std::string root_linkname = "root_link");
void SetJntOffsets(mjModel* m);

mjModel* LoadModel(const char* file, mj::Simulate& sim);
void PreStep(mj::Simulate& sim);
void DoStep(mj::Simulate& sim, 
    std::chrono::time_point<mj::Simulate::Clock> syncCPU,
    mjtNum syncSim); // single sim step
void PhysicsLoop(mj::Simulate& sim);
void InitSimulation(mj::Simulate* sim, const char* mj_filename, const char* xbot_config_path);
void ClearSimulation();
void SimulationLoop(mj::Simulate* sim, const char* mj_filename, const char* xbot_config_path);
void RenderingLoop(mj::Simulate* sim,ros::NodeHandle nh);
void Reset(mj::Simulate& sim);

// close everything
void close();

void reset(mj::Simulate& sim);

// run event loop
void run(const char* fname, const std::string xbot2_config_path,
    ros::NodeHandle nh,
    bool headless = false);
}

#endif // SIMULATOR_H
