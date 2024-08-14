#ifndef __XBOT2_BT_JOINT_H__
#define __XBOT2_BT_JOINT_H__

#include <xbot2/hal/dev_joint.h>
#include <xbot2/client_server/server_manager.h>
#include <mujoco/mujoco.h>

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


private:

    ServerManager::UniquePtr _srv;
    mjModel * _m;
    std::vector<JointInstanceMj::Ptr> _joints;

};

}

#endif
