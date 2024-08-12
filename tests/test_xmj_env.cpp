#include "../src/xmj_sim_env.h"
#include "string.h"
#include <gtest/gtest.h>

class SteppingTest : public ::testing::Test {
protected:

    SteppingTest(std::string xbot2_cfg_path="", 
        const char* modelfilename = "",
        bool headless = true,
        bool multithreaded = true) : 
        xbot2_cfg_path(xbot2_cfg_path),
        headless(headless),
        multithreaded(multithreaded){
        
        mju_strncpy(modelfname, modelfilename, 1000);

        xbot_mujoco_env_ptr = std::make_unique<XBotMjSimEnv>(xbot2_cfg_path,modelfname,
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

TEST_F(SteppingTest, JustSomeSteps) {
    
    int n_steps = 10000;
    int n_steps_done=0;
    for (int i = 0; i < n_steps; i++) {
        xbot_mujoco_env_ptr->step();
        printf("[test_xmj_env]: step idx: %i\n",i);
        n_steps_done++;
    }

    EXPECT_EQ(n_steps_done, n_steps);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

}
