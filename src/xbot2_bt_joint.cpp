#include "xbot2_bt_joint.h"

using namespace XBot;

JointBtServer::JointBtServer(mjModel * mj_model)
{
    for(int i = 0; i < mj_model->njnt; i++)
    {
        if(mj_model->jnt_type[i] != mjtJoint::mjJNT_HINGE &&
               mj_model->jnt_type[i] != mjtJoint::mjJNT_SLIDE)
        {
            continue;
        }

        std::string jname = &mj_model->names[mj_model->name_jntadr[i]];

        auto j = std::make_shared<JointInstanceBt>(
                     Hal::DeviceInfo{jname, "joint_mj", i}
                     );

        j->rx().pos_ref = j->rx().link_pos;
        j->rx().gain_kp = 500;
        j->rx().gain_kd = 10;
        j->tx().reset(j->rx());

        joints.push_back(j);
    }

    std::vector<Hal::DeviceRt::Ptr> devs(joints.begin(), joints.end());

    _srv = std::make_unique<ServerManager>(devs, "sock", "joint_gz");
}

void JointBtServer::run()
{
    for(auto& j : joints)
    {
        j->sense();
    }

    _srv->send();

    _srv->run();

    for(auto& j : joints)
    {
        j->move();
    }

}

JointInstanceBt::JointInstanceBt(Hal::DeviceInfo devinfo):
    BaseType(devinfo)
{

}

bool JointInstanceBt::sense()
{
    _rx.pos_ref = _tx.pos_ref;
    _rx.vel_ref = _tx.vel_ref;
    _rx.tor_ref = _tx.tor_ref;
    _rx.gain_kp = _tx.gain_kp;
    _rx.gain_kd = _tx.gain_kd;

    _rx.motor_pos = _rx.link_pos;
    _rx.motor_vel = _rx.link_vel;

    return true;
}

bool JointInstanceBt::move()
{
    return true;
}

double JointInstanceBt::pid_torque()
{
    double qerr = _tx.pos_ref - _rx.link_pos;
    double dqerr = _tx.vel_ref - _rx.link_vel;

    return _tx.gain_kp*qerr + _tx.gain_kd*dqerr + _tx.tor_ref;
}
