#include "../src/xmj_sim_env.h"
#include "string.h"
#include <gtest/gtest.h>
#include "../src/loading_utils.h"

#include "./config.h"
#define FDIR "@FILES_DIR@"

class SteppingHeadlessTest : public ::testing::Test {
protected:

    SteppingHeadlessTest(bool headless = true,
        bool multithreaded = true) : 
        headless(headless),
        multithreaded(multithreaded){
        

        std::string files_dir(FILES_DIR);

        std::string outfname="centauro_xmj_test";
        LoadingUtils loader(outfname);
        loader.set_urdf_path(files_dir + "/centauro.urdf");
        loader.set_simopt_path(files_dir + "/sim_opt.xml");
        loader.set_world_path(files_dir + "/world.xml");
        loader.set_sites_path(files_dir + "/sites.xml");
        loader.generate();

        std::string mj_xml_path=loader.xml_path();
        std::string mj_xml_content=loader.get_mj_xml();
        std::string xbot2_cfg_path=files_dir+"/xbot2_basic.yaml";

        mju_strncpy(modelfname, mj_xml_path.c_str(), 1000);

        xbot_mujoco_env_ptr = std::make_unique<XBotMjSimEnv>(xbot2_cfg_path,mj_xml_path.c_str(),
            headless,multithreaded);
    }

    void SetUp() override {
        
    }

    void TearDown() override {

        xbot_mujoco_env_ptr->close();

    }

    std::string xbot2_cfg_path;
    char modelfname[1000];
    bool headless;
    bool multithreaded;

    XBotMjSimEnv::UniquePtr xbot_mujoco_env_ptr;

};

class SteppingTest : public ::testing::Test {
protected:

    SteppingTest(bool headless = false,
        bool multithreaded = true) : 
        headless(headless),
        multithreaded(multithreaded){
        
        std::string files_dir(FILES_DIR);

        std::string outfname="centauro_xmj_test";
        LoadingUtils loader(outfname);
        loader.set_urdf_path(files_dir + "/centauro.urdf");
        loader.set_simopt_path(files_dir + "/sim_opt.xml");
        loader.set_world_path(files_dir + "/world.xml");
        loader.set_sites_path(files_dir + "/sites.xml");
        loader.generate();

        std::string mj_xml_path=loader.xml_path();
        std::string mj_xml_content=loader.get_mj_xml();
        std::string xbot2_cfg_path=files_dir+"/xbot2_basic.yaml";

        mju_strncpy(modelfname, mj_xml_path.c_str(), 1000);

        xbot_mujoco_env_ptr = std::make_unique<XBotMjSimEnv>(xbot2_cfg_path,mj_xml_path.c_str(),
            headless,multithreaded);
    }

    void SetUp() override {
        
    }

    void TearDown() override {

        xbot_mujoco_env_ptr->close();

    }

    std::string xbot2_cfg_path;
    char modelfname[1000];
    bool headless;
    bool multithreaded;

    XBotMjSimEnv::UniquePtr xbot_mujoco_env_ptr;

};

TEST_F(SteppingHeadlessTest, JustSomeHeadlessSteps) {
    
    int n_steps = 1000;
    int n_steps_done=0;
    for (int i = 0; i < n_steps; i++) {
        xbot_mujoco_env_ptr->step();
        printf("[test_xmj_env]: step idx: %i\n",i);
        n_steps_done++;
    }

    EXPECT_EQ(n_steps_done, n_steps);
}

// TEST_F(SteppingTest, JustSomeSteps) {
    
//     int n_steps = 10000000;
//     int n_steps_done=0;
//     for (int i = 0; i < n_steps; i++) {
//         xbot_mujoco_env_ptr->step();
//         printf("[test_xmj_env]: step idx: %i\n",i);
//         n_steps_done++;
//     }

//     EXPECT_EQ(n_steps_done, n_steps);
// }

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

}
