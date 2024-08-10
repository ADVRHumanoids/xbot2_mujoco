#ifndef XMJ_SIM_ENV_H
#define XMJ_SIM_ENV_H

#include "mjxmacro.h"
#include "uitools.h"
#include "stdio.h"
#include "string.h"

#include <thread>
#include <mutex>
#include <chrono>

#include "xbot2_bridge.h"
#include "xmj_sim_env_settings.h"

#include <Eigen/Dense>

class XBotMjSimEnv {
public:
    XBotMjSimEnv(const char* configPath);
    ~XBotMjSimEnv();

    void run();
    void prepare();
    static void render(GLFWwindow* window);
    void step();
    void reset();

    static void mj_control_callback(const mjModel* m, mjData* d);
    
    static int uiPredicate(int category, void* userdata);
    static void uiLayout(mjuiState* state);
    static void uiEvent(mjuiState* state);
    static void handleEvent(const mjuiState* state);

    // Static utility functions
    static void dropCallback(GLFWwindow* window, int count, const char** paths);
    static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

    static void drop(GLFWwindow* window, int count, const char** paths);

private:
    

    static XBotMjSimEnv* instance; // Singleton instance pointer

    void xbotmj_control_callback(const mjModel* m, mjData* d);

    void profilerinit();
    void profilerupdate();
    void profilershow(mjrRect rect);
    void sensorinit();
    void sensorupdate();
    void sensorshow(mjrRect rect);
    void infotext(char* title, char* content, double interval);
    void printfield(char* str, void* ptr);
    void watch();
    void makephysics(int oldstate);
    void makerendering(int oldstate);
    void makegroup(int oldstate);
    void makejoint(int oldstate);
    void makecontrol(int oldstate);
    void makesections();

    void simulate();
    void init();
    void alignScale();
    void copyKey();
    static mjtNum timer();
    void clearTimers();
    void updateSettings();
    void loadModel();

    // constants
    const int maxgeom = 5000;           // preallocated geom array in mjvScene
    const double syncmisalign = 0.1;    // maximum time mis-alignment before re-sync
    const double refreshfactor = 0.7;   // fraction of refresh available for simulation

    // Member variables
    mjModel* m = nullptr;
    mjData* d = nullptr;
    
    // abstract visualization
    mjvScene scn;
    mjvCamera cam;
    mjvOption vopt;
    mjvPerturb pert;
    mjvFigure figconstraint;
    mjvFigure figcost;
    mjvFigure figtimer;
    mjvFigure figsize;
    mjvFigure figsensor;

    // OpenGL rendering and UI
    GLFWvidmode vmode;
    int windowpos[2];
    int windowsize[2];
    mjrContext con;
    GLFWwindow* window = NULL;
    mjuiState uistate;
    mjUI ui0, ui1;

    std::mutex mtx;
    std::thread simThread;
    bool running = false;
    char filename[1000];
    double lastUpdateTime = 0;

    // xbot2
    XBot::MjWrapper::UniquePtr xbot2_wrapper;
    std::string xbot2_cfg_path;

};

#endif // XMJ_SIM_ENV_H
