#include "xbot2_mj_joint.h"
#include "xbot2_utils.h"

#include <fnmatch.h>
#include <yaml-cpp/yaml.h>
#include <limits>
#include <chrono>
#include <cstdint>
#include <cstring>

using namespace XBot;

JointMjServer::JointMjServer(mjModel * mj_model, std::string cfg_path):
    _m(mj_model),
    _diag_pub("joint_mj/rtt", "xbot2_mujoco")
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

        // fill torque limits
        j->_tau_max = _m->jnt_actfrcrange[2*i+1];
        
        // initialize pose ref and gains
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
                printf("joint %s setting gains %f, %f\n", jname.c_str(), gains.first, gains.second);
            }
        }

        j->tx().reset(j->rx());

        _joints.push_back(j);
    }

    std::vector<Hal::DeviceRt::Ptr> devs(_joints.begin(), _joints.end());

    _srv = std::make_unique<ServerManager>(devs, "sock", "joint_gz");
}

void JointMjServer::run(mjData * d)
{
    // get microseconds of current time, first 20 bits
    uint64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                Clock::now().time_since_epoch()).count() & 0xFFFFFu;

    // fill rx
    for(auto& j : _joints)
    {
        int ji = j->get_id();
        int qi = _m->jnt_qposadr[ji];
        int vi = _m->jnt_dofadr[ji];

        
        double link_pos_patched = d->qpos[qi];
#ifdef ENABLE_DIAGNOSTICS
        // patch link_pos with current timestamp (in microseconds, modulo 2^20) in the least significant bits, 
        // to measure round trip time 
        // this reduces precision from ~16 to ~10 significant digits (plenty for joint positions)
        utils::patch_double(link_pos_patched, now_us);
#endif
        j->rx().stamp = d->time * 1e6;
        j->rx().link_pos = link_pos_patched;
        j->rx().link_vel = d->qvel[vi];
        j->rx().motor_pos = d->qpos[qi];
        j->rx().motor_vel = d->qvel[vi];
        j->rx().torque = d->qfrc_applied[vi];
        j->sense();
    }

    // send to client
    _srv->send();

    // receive tx
    _srv->run();

    // fill ctrl
    for(auto& j : _joints)
    {
        int ji = j->get_id();
        int vi = _m->jnt_dofadr[ji];

#ifdef ENABLE_DIAGNOSTICS
        // measure round trip delay (note: needs filters OFF on xbot2!)
        if(j->get_name() == "hip_roll_1")
        {
            double pos_ref = j->tx().pos_ref; 
            auto [stamp_us, delay_us] = utils::decode_stamp_and_delay(pos_ref, now_us);
            if(stamp_us != _last_recv_stamp)
            {
                _rtt_acc.update(delay_us);
                _last_recv_stamp = stamp_us;
            }
        }
#endif
        
        d->qfrc_applied[vi] = j->pid_torque();
        j->move();
    }

#ifdef ENABLE_DIAGNOSTICS
    // publish diagnostics every 1 second
    auto now = Clock::now();
    if(now - _last_pub_time > 1s)
    {
        _diag_pub.publish_stats("rtt", _rtt_acc);
        _last_pub_time = now;   
    }
#endif
}

JointInstanceMj::JointInstanceMj(Hal::DeviceInfo devinfo):
    BaseType(devinfo)
{
    _tau_max = std::numeric_limits<double>::infinity();
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

    double tau = _tx.gain_kp*qerr + _tx.gain_kd*dqerr + _tx.tor_ref;
    return std::max(-_tau_max, std::min(tau, _tau_max));
}

