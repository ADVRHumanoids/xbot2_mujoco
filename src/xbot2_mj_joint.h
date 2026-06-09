#ifndef __XBOT2_BT_JOINT_H__
#define __XBOT2_BT_JOINT_H__

#include <xbot2/hal/dev_joint.h>
#include <xbot2/client_server/server_manager.h>
#include <mujoco/mujoco.h>

#ifdef ENABLE_DIAGNOSTICS
#include <xbot2_diagnostics/publisher.h>
#endif

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

    double _tau_max;

    int _last_stamp = -1;

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

#ifdef ENABLE_DIAGNOSTICS
    using Clock = std::chrono::steady_clock;
    diagnostics::DiagPublisher _diag_pub;
    diagnostics::StatAccumulator _rtt_acc;
    int _last_recv_stamp = -1;
    Clock::time_point _last_pub_time;
#endif

};

}

#endif
