#include "string.h"
#include <gtest/gtest.h>
#include <chrono>
#include <vector>
#include <cmath>  // For sin and cos
#include <random>

#include "../src/xmj_sim_env.h"
#include "../src/loading_utils.h"
#include "./config.h"

void quaternion_from_rotation_z(double theta_degrees, std::vector<double>& quaternion) {
    // Convert theta from degrees to radians
    double theta_radians = theta_degrees * M_PI / 180.0;

    // Calculate the quaternion components for z-axis rotation
    quaternion[0] = std::cos(theta_radians / 2.0);  // w
    quaternion[1] = 0.0;                            // x
    quaternion[2] = 0.0;                            // y
    quaternion[3] = std::sin(theta_radians / 2.0);  // z
}

class ManualSteppingTest : public ::testing::TestWithParam<std::tuple<bool, bool, int, int>> {
protected:

    ManualSteppingTest(){
        
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

        xbot_mujoco_env_ptr = std::make_unique<XBotMjSimEnv>(
            mj_xml_path.c_str(),
            ros_nh,
            xbot2_cfg_path,
            std::get<0>(GetParam()),
            std::get<1>(GetParam()),
            std::get<2>(GetParam()),
            std::get<3>(GetParam()));
    }

    void SetUp() override {
        
    }

    void TearDown() override {


    }

    XBotMjSimEnv::UniquePtr xbot_mujoco_env_ptr;

};

TEST_P(ManualSteppingTest, TestSim) {

    // Create a random device to seed the random number generator
    std::random_device rd;
    // Create a Mersenne Twister random number generator
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-180.0, 180.0);
    std::uniform_real_distribution<> dis_pos(-1.0, 1.0);

    int actual_done = xbot_mujoco_env_ptr->step_counter;
    double target_stime=5.0;

    int n_steps = 20000;
    auto start = std::chrono::high_resolution_clock::now();
    int reset_freq = n_steps/8;
    for (int i = 0; i < n_steps; ++i) {
        if (!xbot_mujoco_env_ptr->step()) {
            break;
        }
        if ((i+1)%reset_freq==0) {
            xbot_mujoco_env_ptr->p_i[0] = xbot_mujoco_env_ptr->p_i[0]+dis_pos(gen);
            xbot_mujoco_env_ptr->p_i[1] = xbot_mujoco_env_ptr->p_i[1]+dis_pos(gen);
            double random_theta= dis(gen);
            quaternion_from_rotation_z(random_theta, xbot_mujoco_env_ptr->q_i);
            xbot_mujoco_env_ptr->reset();
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> walltime = end - start;

    xbot_mujoco_env_ptr->close(); // close everything
    actual_done=xbot_mujoco_env_ptr->step_counter-actual_done;

    double physics_dt = xbot_mujoco_env_ptr->physics_dt;
    double simtime_elapsed = physics_dt*actual_done;

    printf("[test_xmj_env][ManualSteppingTest]: n of timesteps done %i VS %i\n", actual_done, n_steps);
    printf("[test_xmj_env][ManualSteppingTest]: elapsed wall time %f [s] VS simulated time %f [s]. \n RT factor %f, physics dt %f \n", 
        walltime.count(), simtime_elapsed, simtime_elapsed/walltime.count(),physics_dt);

    EXPECT_EQ(actual_done, n_steps); 
}

// Instantiate the parameterized test case with different parameters
INSTANTIATE_TEST_SUITE_P(
    ManualSteppingTestCases,
    ManualSteppingTest,
    ::testing::Values(
        std::make_tuple(false, // headless
            true, // manual stepping 
            100, // n of initial steps
            1000 // timeout [ms]
            )
    )
);

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "XMjEnvTests");

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

}
