#include <pybind11/pybind11.h>

#include "../../src/xmj_sim_env.h"

namespace py = pybind11;

inline bool isRelease() {
    #ifdef IS_RELEASE
        return true;
    #else
        return false;
    #endif
}

PYBIND11_MODULE(PyXbotMjSimEnv, m) {

    m.doc() = "pybind11 for xbot2_mujoco's XbotMjSimEnv class";

    py::class_<XBotMjSimEnv>(m, "XBotMjSimEnv")
            .def(py::init<const std::string, ros::NodeHandle, const std::string, bool, bool, int, int>(),  
                py::arg("model_fname"), py::arg("nh"), py::arg("xbot2_config_path")="", 
                py::arg("headless") = false, py::arg("manual_stepping") = true,
                py::arg("init_steps") = 1, py::arg("timeout") = 10)
            .def("reset", &XBotMjSimEnv::reset)
            .def("close", &XBotMjSimEnv::close)
            .def("step", &XBotMjSimEnv::step)
            .def("is_running", &XBotMjSimEnv::is_running)
            ;

}

