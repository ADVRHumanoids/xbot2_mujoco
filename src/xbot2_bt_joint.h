#ifndef __XBOT2_BT_JOINT_H__
#define __XBOT2_BT_JOINT_H__

#include <xbot2/hal/dev_joint.h>
#include <xbot2/client_server/server_manager.h>
#include "mujoco.h"

namespace XBot
{

class JointInstanceBt : public Hal::DeviceTplCommon<Hal::joint_rx,
        Hal::joint_tx>
{

public:

    XBOT2_DECLARE_SMART_PTR(JointInstanceBt)

    typedef DeviceTplCommon<Hal::joint_rx,
    Hal::joint_tx> BaseType;

    JointInstanceBt(Hal::DeviceInfo devinfo);

    bool sense() override;

    bool move() override;

    double pid_torque();

private:


};

class JointBtServer
{

public:

    XBOT2_DECLARE_SMART_PTR(JointBtServer);

    JointBtServer(mjModel * mj_model);

    void run();

    std::vector<JointInstanceBt::Ptr> joints;

private:

    ServerManager::UniquePtr _srv;

};

}

#endif
