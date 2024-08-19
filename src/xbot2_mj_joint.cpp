#include "xbot2_mj_joint.h"
#include <fnmatch.h>

using namespace XBot;

JointMjServer::JointMjServer(mjModel * mj_model, std::string cfg_path):
    _m(mj_model)
{

    YAML::Node cfg;
    if(!cfg_path.empty())
    {
        cfg = YAML::LoadFile(cfg_path);
    }

    auto motor_pd = cfg["motor_pd"];

    for(int i = 0; i < mj_model->njnt; i++)
    {
        if(_m->jnt_type[i] != mjtJoint::mjJNT_HINGE &&
               _m->jnt_type[i] != mjtJoint::mjJNT_SLIDE)
        {
            continue;
        }

        std::string jname = &_m->names[mj_model->name_jntadr[i]];

        auto j = std::make_shared<JointInstanceMj>(
                     Hal::DeviceInfo{jname, "joint_mj", i}
                     );
                     
        j->rx().pos_ref = j->rx().link_pos;
        j->rx().gain_kp = 100;
        j->rx().gain_kd = 5;

        for(auto pair: motor_pd)
        {
            auto pattern = pair.first.as<std::string>();
            auto gains = pair.second.as<std::pair<double, double>>();

            if(fnmatch(pattern.c_str(), jname.c_str(), 0) == 0)
            {
                j->rx().gain_kp = gains.first;
                j->rx().gain_kd = gains.second;

                motor_pd_map[jname] = std::make_pair(gains.first,gains.second);
            }
        }

        printf("[XBot][JointMjServer]: constructed xbot joint %s \n with impedance gains kp: %f, kd: %f\n and pos ref %f\n",
            jname.c_str(),j->rx().gain_kp,j->rx().gain_kd,j->rx().pos_ref);

        j->tx().reset(j->rx());

        _joints.push_back(j);
        _mj_jnt_names.push_back(jname);
    }

    // (defaults to 0 if mj joint is not in the SRDF homing group)
    // _print_homing_config(); // db print

    std::vector<Hal::DeviceRt::Ptr> devs(_joints.begin(), _joints.end());

    _srv = std::make_unique<ServerManager>(devs, "sock", "joint_gz");
}

void JointMjServer::reset(mjData * d)
{
    int qi = 0.0;
    double current_pos = 0.0;
    for(int i = 0; i < _joints.size(); i++)
    {
        qi = _m->jnt_qposadr[_joints[i]->get_id()];
        current_pos = d->qpos[qi];
        _joints[i]->reset(current_pos,
            motor_pd_map[_mj_jnt_names[i]].first,
            motor_pd_map[_mj_jnt_names[i]].second);
    
    }

}
void JointMjServer::run(mjData * d)
{
    for(auto& j : _joints)
    {
        int qi = _m->jnt_qposadr[j->get_id()];
        int vi = _m->jnt_dofadr[j->get_id()];
        j->rx().link_pos = d->qpos[qi];
        j->rx().link_vel = d->qvel[vi];
        j->rx().motor_pos = d->qpos[qi];
        j->rx().motor_vel = d->qvel[vi];
        j->rx().torque = j->pid_torque();
        j->sense();
    }

    _srv->send();

    _srv->run();
        
    for(auto& j : _joints)
    {
        int vi = _m->jnt_dofadr[j->get_id()];
        d->ctrl[vi] = j->rx().torque;
        j->move();
    }

}

JointInstanceMj::JointInstanceMj(Hal::DeviceInfo devinfo):
    BaseType(devinfo)
{

}

bool JointInstanceMj::sense()
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

bool JointInstanceMj::move()
{
    return true;
}

void JointInstanceMj::reset(const double p_ref, const double kp, const double kd)
{
    _tx.pos_ref = p_ref;
    _tx.vel_ref = 0.0;
    _tx.tor_ref = 0.0;
    _tx.gain_kp = kp;
    _tx.gain_kd = kd;

}

double JointInstanceMj::pid_torque()
{
    double qerr = _tx.pos_ref - _rx.link_pos;
    double dqerr = _tx.vel_ref - _rx.link_vel;

    return _tx.gain_kp*qerr + _tx.gain_kd*dqerr + _tx.tor_ref;
}

