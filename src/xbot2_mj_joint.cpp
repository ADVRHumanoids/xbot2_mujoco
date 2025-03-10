#include "xbot2_mj_joint.h"
#include <fnmatch.h>
#include <yaml-cpp/yaml.h>

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
                printf("joint %s setting gains %f, %f\n", jname.c_str(), gains.first, gains.second);
            }
        }

        j->tx().reset(j->rx());

        _joints.push_back(j);
    }

    std::vector<Hal::DeviceRt::Ptr> devs(_joints.begin(), _joints.end());

    _srv = std::make_unique<ServerManager>(devs, "sock", "joint_gz");

    // _logger = MatLogger2::MakeLogger("/tmp/joint_mj_server_log");
    // _logger->set_buffer_mode(VariableBuffer::Mode::circular_buffer);
}

void JointMjServer::run(mjData * d)
{
    // _logger->add("time", d->time);

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

    // _logger->add("q", Eigen::Map<Eigen::VectorXd>(d->qpos, _m->nq));

    // send rx data (telemetry)
    // it also contains tx feedback, i.e. the tx that generated
    // the last ctrl (at prev iteration)
    _srv->send();

    // receive tx data (commands)
    _srv->run();

    Eigen::VectorXd qref(_m->nv);

    for(auto& j : _joints)
    {
        int vi = _m->jnt_dofadr[j->get_id()];
        d->ctrl[vi] = j->rx().torque;
        qref[vi] = j->tx().pos_ref;
        j->move();
    }

    // _logger->add("qref", qref);

    // int wb = _logger->flush_available_data();
    // if(wb) printf("flushed %d bytes\n", wb);
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

