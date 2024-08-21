// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "simulator.h"

using namespace xbot_mujoco;

// run the full simulation loop
int main(int argc, char** argv)
{

    std::string xbot2_cfg_path;
    char filename[mj::Simulate::kMaxFilenameLength];

    // request loadmodel if file given (otherwise drag-and-drop)
    if( argc>1 )
    {
        mju_strncpy(filename, argv[1], 1000);
    }

    // xbot2 config
    if( argc>2 )
    {
        xbot2_cfg_path = argv[2];
    }

    bool headless=false;
    ros::init(argc, argv, "mujoco_ros");
    ros::NodeHandle nh("");
    run(filename,xbot2_cfg_path,nh,headless); // run everything
    
    return 0;
}
