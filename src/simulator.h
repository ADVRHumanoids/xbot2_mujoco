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

#include "mjxmacro.h"
#include "uitools.h"
#include "stdio.h"
#include "string.h"

#include <thread>
#include <mutex>
#include <chrono>

#include "simulator_settings.h"
#include "xbot2_bridge.h"

#include <Eigen/Dense>

//-------------------------------- global -----------------------------------------------

// OpenGL rendering and UI
static GLFWvidmode vmode;
static int windowpos[2];
static int windowsize[2];
static mjrContext con;
static GLFWwindow* window = NULL;
static mjuiState uistate;
static mjUI ui0, ui1;

// model and data
static mjModel* m = NULL;
static mjData* d = NULL;
static char filename[1000] = "";

// sim thread synchronization
static std::mutex mtx;

// abstract visualization
static mjvScene scn;
static mjvCamera cam;
static mjvOption vopt;
static mjvPerturb pert;
static mjvFigure figconstraint;
static mjvFigure figcost;
static mjvFigure figtimer;
static mjvFigure figsize;
static mjvFigure figsensor;

// xbot2
static XBot::MjWrapper::UniquePtr xbot2_wrapper;
static std::string xbot2_cfg_path;

void xbotmj_control_callback(const mjModel* m, mjData* d);

//----------------------- profiler, sensor, info, watch ---------------------------------

// init profiler figures
void profilerinit(void);

// update profiler figures
void profilerupdate(void);

// show profiler figures
void profilershow(mjrRect rect);

// init sensor figure
void sensorinit(void);

// update sensor figure
void sensorupdate(void);

// show sensor figure
void sensorshow(mjrRect rect);

// prepare info text
void infotext(char* title, char* content, double interval);

// sprintf forwarding, to avoid compiler warning in x-macro
void printfield(char* str, void* ptr);

// update watch
void watch(void);


//-------------------------------- UI construction --------------------------------------

// make physics section of UI
void makephysics(int oldstate);

// make rendering section of UI
void makerendering(int oldstate);

// make group section of UI
void makegroup(int oldstate);

// make joint section of UI
void makejoint(int oldstate);
 
// make control section of UI
void makecontrol(int oldstate);
 
// make model-dependent UI sections
void makesections(void);

//-------------------------------- utility functions ------------------------------------

// align and scale view
void alignscale(void);

// copy qpos to clipboard as key
void copykey(void);

// millisecond timer, for MuJoCo built-in profiler
mjtNum timer(void);

// clear all times
void cleartimers(void);

// update UI 0 when MuJoCo structures change (except for joint sliders)
void updatesettings(void);

// drop file callback
void drop(GLFWwindow* window, int count, const char** paths);

// load mjb or xml model
void loadmodel(void);

//--------------------------------- UI hooks (for uitools.c) ----------------------------

// determine enable/disable item state given category
int uiPredicate(int category, void* userdata);

// set window layout
void uiLayout(mjuiState* state);

// handle UI event
void uiEvent(mjuiState* state);

//--------------------------- rendering and simulation ----------------------------------

// prepare to render
void prepare(void);

// simulate in background thread (while rendering in main thread)
bool exit_requested(void);
void require_exit(void);
void step(double cpusync, mjtNum simsync); // just a single step
void simulate(void); 

//-------------------------------- init, control callback and sim loop run ----------------------------------------

// initalize everything
void init(bool headless = false);

// run event loop
void run(const char* fname, const std::string xbot2_config_path,
    bool headless = false);

#endif // SIMULATOR_H
