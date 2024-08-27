#ifndef __XBOT2_BT_JOINT_H__
#define __XBOT2_BT_JOINT_H__

#include <xbot2/hal/dev_joint.h>
#include <xbot2/client_server/server_manager.h>
#include <mujoco/mujoco.h>

#include <yaml-cpp/yaml.h>

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

    void reset(const double p_ref, const double kp, const double kd);

private:


};

class JointMjServer
{

public:

    XBOT2_DECLARE_SMART_PTR(JointMjServer);

    JointMjServer(mjModel * mj_model, std::string cfg_path);

    void run(mjData * d);

    void reset(mjData * d);

private:

    ServerManager::UniquePtr _srv;
    mjModel * _m;
    std::vector<JointInstanceMj::Ptr> _joints;

    std::vector<std::string> _mj_jnt_names;

    std::map<std::string, std::pair<double, double>> motor_pd_map;

    double default_kp = 1000;
    double default_kd = 20;
    double default_small_kp = 10;
    double default_small_kd = 1;
    double default_kp_arms = 200;
    double default_kd_arms = 10;
    double default_kd_wheels = 30;

};

}

#endif
