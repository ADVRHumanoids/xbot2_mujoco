#include "string.h"
#include <gtest/gtest.h>

#include "../src/xmj_sim_env.h"
#include "../src/loading_utils.h"
#include "./config.h"

class SimRunTest : public ::testing::TestWithParam<std::tuple<bool, bool>> {
protected:

    SimRunTest(){
        
        ros::NodeHandle ros_nh("");

        std::string mesh_root_directory = std::string(IIT_CENTAURO_ROS_PKG_ROOT)+std::string("/centauro_urdf/meshes");
        std::vector<std::string> subdirs = {"v2", "realsense", "simple"}; // list of subdirs where
        // meshes may be located

        std::string outfname="XMjEnvTest";
        std::string files_dir(FILES_DIR);
        std::string xbot2_cfg_path=files_dir+"/xbot2_basic.yaml";

        LoadingUtils loader(outfname);
        loader.set_mesh_rootdir(mesh_root_directory);
        loader.set_mesh_rootdir_subdirs(subdirs);
        loader.set_urdf_path(files_dir + "/centauro.urdf");
        loader.set_simopt_path(files_dir + "/sim_opt.xml");
        loader.set_world_path(files_dir + "/world.xml");
        loader.set_sites_path(files_dir + "/sites.xml");
        loader.set_xbot_config_path(xbot2_cfg_path);
        loader.generate();

        std::string mj_xml_path=loader.xml_path();
        std::string mj_xml_content=loader.get_mj_xml();

        xbot_mujoco_env_ptr = std::make_unique<XBotMjSimEnv>(xbot2_cfg_path,
            mj_xml_path.c_str(),
            ros_nh,
            std::get<0>(GetParam()),
            std::get<1>(GetParam()));
    }

    void SetUp() override {
        
    }

    void TearDown() override {

        xbot_mujoco_env_ptr->close();

    }

    XBotMjSimEnv::UniquePtr xbot_mujoco_env_ptr;

};

TEST_P(SimRunTest, TestSim) {
    xbot_mujoco_env_ptr->run();
}

// Instantiate the parameterized test case with different parameters
INSTANTIATE_TEST_SUITE_P(
    SimRunTestCases,
    SimRunTest,
    ::testing::Values(
        std::make_tuple(false, true)
    )
);

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "XMjEnvTests");

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

}
