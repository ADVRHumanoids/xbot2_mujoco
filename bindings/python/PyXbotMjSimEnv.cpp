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
            .def(py::init<const std::string, const char*, ros::NodeHandle, bool, bool>(),  py::arg("xbot2_cfg_path"), py::arg("model_fname") = "", py::arg("ros_nh"), py::arg("headless") = false, py::arg("multithread") = true)
            .def("render_window", &XBotMjSimEnv::render_window)
            .def("reset", &XBotMjSimEnv::reset)
            .def("close", &XBotMjSimEnv::close)
            ;

}

