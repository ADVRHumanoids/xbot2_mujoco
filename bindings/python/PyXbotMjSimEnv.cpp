#include <pybind11/pybind11.h>

inline bool isRelease() {
    #ifdef IS_RELEASE
        return true;
    #else
        return false;
    #endif
}

PYBIND11_MODULE(PyXbotMjSimEnv, m) {

    m.doc() = "pybind11 for xbot2_mujoco's XbotMjSimEnv class";

}

