#ifndef LOADING_UTILS_H
#define LOADING_UTILS_H

#include <string>
#include <pugixml.hpp>
#include <vector>
#include <yaml-cpp/yaml.h>  // Include the yaml-cpp header
#include <map>

class LoadingUtils {
public:
    typedef std::unique_ptr<LoadingUtils> UniquePtr;
    typedef std::shared_ptr<LoadingUtils> Ptr;

    LoadingUtils(const std::string& name = "XBot2Mujoco_LoadingUtils");

    // setters
    void set_mesh_rootdir(const std::string& mesh_root_dir="None");
    void set_mesh_rootdir_subdirs(const std::vector<std::string>& mesh_rootsubdirs);
    void set_urdf_path(const std::string& urdfpath);
    void set_srdf_path(const std::string& srdfpath);
    void set_urdf_cmd(const std::string& urdfcmd);
    void set_simopt_path(const std::string& simoptpath);
    void set_world_path(const std::string& worldpath);
    void set_sites_path(const std::string& sitespath);
    void set_xbot_config_path(const std::string& configpath);
    std::string get_xbot_config_path();

    // homing-related utilities
    static std::string get_srdf_path_fromxbotconfig(std::string xbot_cf_path);
    static std::string get_urdf_path_fromxbotconfig(std::string xbot_cf_path);
    static std::map<std::string, double> get_homing_from_srdf(const std::string& srdf_path);
    static std::map<std::string, double> generate_homing_map(const std::vector<std::string>& jnt_name_list,
        std::string srdfpath="",
        const std::string xbot_cf_path="", double fallback_val = 0.0);
    static std::map<std::string, double> generate_homing_map(const std::string xbot_cf_path="",
        std::string srdfpath="");
    static std::vector<double> generate_homing_from_list(const std::vector<std::string>& jnt_name_list,
        std::string srdfpath="",
        const std::string xbot_cf_path="", double fallback_val = 0.0);
    static std::tuple<std::vector<std::string>, std::vector<double>> generate_ordered_homing(std::string srdfpath="",
        const std::string xbot_cf_path="");
    static void print_homing(std::map<std::string, double> homing_map);

    // getters
    std::string get_mj_xml(); // Public method to get the final MuJoCo XML
    void generate();
    std::string xml_path();

private:

    bool generated = false;
    bool use_custom_mesh_rootdir = false;
    std::string name;

    std::string mesh_rootdir;
    std::vector<std::string> mesh_root_subdirs;

    std::string urdf_path;
    std::string srdf_path;
    std::string urdf_command;
    std::string simopt_path;
    std::string world_path;
    std::string sites_path;

    std::string xbot_config_path;

    std::string mjxml_dir;
    std::string mjurdf_path;
    std::string mjxml_path;
    std::string mjxml_path_orig;

    std::string remove_comments(const std::string& xml);
    std::string apply_mesh_root(const std::string& urdf, const std::string& mesh_rootdir,
        const std::vector<std::string>& subdir_names);

    std::string add_mesh_simlink_bfix(const std::string& urdf);
    void preprocess_urdf();
    void compile_urdf_for_mj(std::string xml, std::string outpath);
    void add_compiler_opts();
    void merg_xml_trees(pugi::xml_node& parent, pugi::xml_node child);
    void merge_xml(std::string to_be_added, std::string into, std::string out);
    void add_sites();
};

#endif // LOADING_UTILS_H