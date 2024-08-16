#ifndef __XBOT2_BT_JOINT_H__
#define __XBOT2_BT_JOINT_H__

#include <xbot2/hal/dev_joint.h>
#include <xbot2/client_server/server_manager.h>
#include <mujoco/mujoco.h>
#include "loading_utils.h"

namespace XBot
{

class JointInstanceMj : public Hal::DeviceTplCommon<Hal::joint_rx,
        Hal::joint_tx>
{

public:

    XBOT2_DECLARE_SMART_PTR(JointInstanceMj)

    typedef DeviceTplCommon<Hal::joint_rx,
    Hal::joint_tx> BaseType;

    JointInstanceMj(Hal::DeviceInfo devinfo);

    bool sense() override;

    bool move() override;

    double pid_torque();

private:


};

class JointMjServer
{

public:

    XBOT2_DECLARE_SMART_PTR(JointMjServer);

    JointMjServer(mjModel * mj_model, std::string cfg_path);

    void run(mjData * d);

    void move_to_homing_now(mjData * d);

private:

    LoadingUtils::UniquePtr _loader_ptr;

    ServerManager::UniquePtr _srv;
    mjModel * _m;
    std::vector<JointInstanceMj::Ptr> _joints;

    std::vector<std::string> _mj_jnt_names;

    std::map<std::string, double> _homing_map;

    void _set_model_homing();

};

}

#endif
