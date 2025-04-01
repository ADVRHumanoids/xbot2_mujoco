#include <pybind11/pybind11.h>
#include <pybind11/stl.h>  // Include this to handle STL types like std::vector and std::string

#include "../../src/xmj_sim.h"
#include "../../src/loading_utils.h"

#include <pybind11/numpy.h>
#include <vector>

namespace py = pybind11;

py::array_t<double> vector_to_numpy(const std::vector<double>& vec) {
    return py::array_t<double>(
        { vec.size() },                  // shape
        { sizeof(double) },              // strides
        vec.data() // data pointer
    );
}

void numpy_to_vector(const pybind11::array_t<double>& arr, std::vector<double>& vec) {
    auto buf = arr.request();
    vec.assign(static_cast<double*>(buf.ptr), static_cast<double*>(buf.ptr) + buf.size);
}

inline bool isRelease() {
    #ifdef IS_RELEASE
        return true;
    #else
        return false;
    #endif
}

PYBIND11_MODULE(PyXbotMjSim, m) {

    m.doc() = "pybind11 bindings for xbot2_mujoco's XBotMjSim class";

    py::class_<XBotMjSim>(m, "XBotMjSim")
        // Bind the constructor
        .def(py::init<const std::string, const std::string, bool, bool, int, int, const std::string, bool, double>(),
             py::arg("model_fname"), py::arg("xbot2_config_path") = "",
             py::arg("headless") = false, py::arg("manual_stepping") = true,
             py::arg("init_steps") = 1, py::arg("timeout") = 10,
             py::arg("base_link_name") = "base_link",
             py::arg("match_rt_factor") = true,
             py::arg("rt_factor_trgt") = 1.0)

        // Bind the public methods
        .def("reset", &XBotMjSim::reset, "Reset the simulation environment")
        .def("close", &XBotMjSim::close, "Close the simulation environment")
        .def("step", &XBotMjSim::step, "Perform one simulation step")
        .def("is_running", &XBotMjSim::is_running, "Check if the simulation is running")
        .def("n_jnts", &XBotMjSim::n_jnts, "get n of controllable joints")
        .def("jnt_names", &XBotMjSim::jnt_names, "get names of controllable joints")
        .def("move_to_homing_now", &XBotMjSim::jnt_names, "move robot to homing NOW!")
        .def("move_base_to_now", &XBotMjSim::jnt_names, "move robot base somewhere NOW!")

        // Bind the public attributes
        .def_readwrite("base_link_name", &XBotMjSim::base_link_name, "Name of the base link")
        .def_readwrite("step_counter", &XBotMjSim::step_counter, "Step counter")
        .def_readwrite("physics_dt", &XBotMjSim::physics_dt, "Physics time step")

        .def("get_pi", [](const XBotMjSim& self) { return vector_to_numpy(self.p_i); })
        .def("set_pi", [](XBotMjSim& self, const pybind11::array_t<double>& arr) {numpy_to_vector(arr, self.p_i);})
        .def("get_qi", [](const XBotMjSim& self) { return vector_to_numpy(self.q_i); })
        .def("set_qi", [](XBotMjSim& self, const pybind11::array_t<double>& arr) {numpy_to_vector(arr, self.q_i);})
        .def_property_readonly("p", [](const XBotMjSim& self) { return vector_to_numpy(self.p); })
        .def_property_readonly("q", [](const XBotMjSim& self) { return vector_to_numpy(self.q); })
        .def_property_readonly("twist", [](const XBotMjSim& self) { return vector_to_numpy(self.twist); })
        .def_property_readonly("jnts_q", [](const XBotMjSim& self) { return vector_to_numpy(self.jnts_q); })
        .def_property_readonly("jnts_v", [](const XBotMjSim& self) { return vector_to_numpy(self.jnts_v); })
        .def_property_readonly("jnts_a", [](const XBotMjSim& self) { return vector_to_numpy(self.jnts_a); })
        .def_property_readonly("jnts_eff", [](const XBotMjSim& self) { return vector_to_numpy(self.jnts_eff); })

        ;
    
    py::class_<LoadingUtils>(m, "LoadingUtils")
        .def(py::init<const std::string&>(), py::arg("name") = "XBot2Mujoco_LoadingUtils")

        // Bind setters
        .def("set_mesh_rootdir", &LoadingUtils::set_mesh_rootdir, py::arg("mesh_root_dir") = "None")
        .def("set_mesh_rootdir_subdirs", &LoadingUtils::set_mesh_rootdir_subdirs, py::arg("mesh_rootsubdirs"))
        .def("set_urdf_path", &LoadingUtils::set_urdf_path, py::arg("urdfpath"))
        .def("set_srdf_path", &LoadingUtils::set_srdf_path, py::arg("urdfpath"))
        .def("set_urdf_cmd", &LoadingUtils::set_urdf_cmd, py::arg("urdfcmd"))
        .def("set_simopt_path", &LoadingUtils::set_simopt_path, py::arg("simoptpath"))
        .def("set_world_path", &LoadingUtils::set_world_path, py::arg("worldpath"))
        .def("set_sites_path", &LoadingUtils::set_sites_path, py::arg("sitespath"))
        .def("set_xbot_config_path", &LoadingUtils::set_xbot_config_path, py::arg("configpath"))

        // Bind getters
        .def("get_xbot_config_path", &LoadingUtils::get_xbot_config_path)
        .def("get_mj_xml", &LoadingUtils::get_mj_xml)
        .def("xml_path", &LoadingUtils::xml_path)
        .def("generate", &LoadingUtils::generate)

        // Bind static methods
        .def_static("get_srdf_path_fromxbotconfig", &LoadingUtils::get_srdf_path_fromxbotconfig, py::arg("xbot_cf_path"))
        .def_static("get_urdf_path_fromxbotconfig", &LoadingUtils::get_urdf_path_fromxbotconfig, py::arg("xbot_cf_path"))
        .def_static("get_homing_from_srdf", &LoadingUtils::get_homing_from_srdf, py::arg("srdf_path"))
        .def_static("generate_homing_map", py::overload_cast<const std::vector<std::string>&, std::string, const std::string, double>(&LoadingUtils::generate_homing_map),
                        py::arg("jnt_name_list"), 
                        py::arg("srdf_path")="",
                        py::arg("xbot_cf_path")="",
                        py::arg("fallback_val") = 0.0)
        .def_static("generate_homing_map", py::overload_cast<std::string,const std::string>(&LoadingUtils::generate_homing_map),
                        py::arg("srdf_path")="",
                        py::arg("xbot_cf_path"))
        .def_static("generate_homing_from_list", &LoadingUtils::generate_homing_from_list, 
                        py::arg("jnt_name_list"), 
                        py::arg("srdf_path")="",
                        py::arg("xbot_cf_path")="", 
                        py::arg("fallback_val") = 0.0)
        .def_static("generate_ordered_homing", &LoadingUtils::generate_ordered_homing, 
                        py::arg("srdf_path")="",
                        py::arg("xbot_cf_path")="")
        .def_static("print_homing", &LoadingUtils::print_homing, py::arg("homing_map"))
        ;
}
