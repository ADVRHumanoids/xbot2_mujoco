#include "../src/xmj_sim_env.h"
#include "string.h"

int main(int argc, const char** argv)
{
    std::string xbot2_cfg_path = "";
    char modelfname[1000] = "";

    if( argc>1 )
    {
        mju_strncpy(modelfname, argv[1], 1000);
    }
    if( argc>2 )
    {
        xbot2_cfg_path = argv[2];
    }

    bool headless = true;
    bool multithreaded = true;
    
    int n_steps = 10000;

    printf("[test_xmj_env]: Will try to load XBot2 config file at %s",xbot2_cfg_path.c_str());
    printf("[test_xmj_env]: Will try to load xml model file %s",modelfname);

    XBotMjSimEnv xbot_mujoco_env = XBotMjSimEnv(xbot2_cfg_path,modelfname,headless,multithreaded);

    for (int i = 0; i < n_steps; i++) {
        xbot_mujoco_env.step();
        printf("[test_xmj_env]: step idx: %i\n",i);
    }
}
