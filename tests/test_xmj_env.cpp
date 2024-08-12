#include "../src/xmj_sim_env.h"
#include "string.h"

int main(int argc, const char** argv)
{
    std::string xbot2_cfg_path = "";
    // xbot2 config
    if( argc>1 )
    {
        xbot2_cfg_path = argv[2];
    }

    printf("[test_xmj_env]: Will try to load XBot2 config file at %s",xbot2_cfg_path.c_str());

    XBotMjSimEnv xbot_mujoco_env = XBotMjSimEnv(xbot2_cfg_path);

}
