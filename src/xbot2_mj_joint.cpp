#include "xbot2_mj_joint.h"
#include <fnmatch.h>
#include <yaml-cpp/yaml.h>

using namespace XBot;

JointMjServer::JointMjServer(mjModel * mj_model, std::string cfg_path):
    _m(mj_model)
{

    _loader_ptr = std::make_unique<LoadingUtils>("xbot2_bridge_loading_utils");
    _loader_ptr->set_xbot_config_path(cfg_path);

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

        fprintf(stderr, "[XBot][JointMjServer]: constructing xbot joint %s \n",jname.c_str());

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
                printf("with impedance gains %f, %f\n", gains.first, gains.second);
            }
        }

        j->tx().reset(j->rx());

        _joints.push_back(j);
        _mj_jnt_names.push_back(jname);
    }

    _homing_map = _loader_ptr->generate_homing_map_from_other(_mj_jnt_names); // retrieved from SRDF
    _set_model_homing(); // writes default homing for joint contained in both SRDF and mujoco's model
    // (defaults to 0 if mj joint is not in the SRDF homing group)

    std::vector<Hal::DeviceRt::Ptr> devs(_joints.begin(), _joints.end());

    _srv = std::make_unique<ServerManager>(devs, "sock", "joint_gz");
}

void JointMjServer::move_to_homing_now(mjData * d) {

    for (const auto& homing_data : _homing_map) {
        std::string xbot_jnt = homing_data.first;

        int joint_id = mj_name2id(_m, mjOBJ_JOINT, xbot_jnt.c_str());
        if (joint_id != -1) {// joint found -> set default joint pos
            int qpos_adr = _m->jnt_qposadr[joint_id];
            d->qpos[qpos_adr] = homing_data.second;
            d->qvel[qpos_adr] = 0.0;
            d->ctrl[qpos_adr] = 0.0;
        }
    }
}

void JointMjServer::_set_model_homing() {

    for (const auto& homing_data : _homing_map) {
        std::string xbot_jnt = homing_data.first;

        int joint_id = mj_name2id(_m, mjOBJ_JOINT, xbot_jnt.c_str());
        if (joint_id != -1) {// joint found -> set default joint pos
            int qpos_adr = _m->jnt_qposadr[joint_id];
            _m->qpos0[qpos_adr] = homing_data.second;
        }
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

double JointInstanceMj::pid_torque()
{
    double qerr = _tx.pos_ref - _rx.link_pos;
    double dqerr = _tx.vel_ref - _rx.link_vel;

    return _tx.gain_kp*qerr + _tx.gain_kd*dqerr + _tx.tor_ref;
}

