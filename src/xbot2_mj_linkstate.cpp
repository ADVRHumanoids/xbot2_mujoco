#include "xbot2_mj_linkstate.h"
using namespace XBot;

LinkStateInstanceMj::LinkStateInstanceMj(Hal::DeviceInfo devinfo):
    BaseType(devinfo)
{

}

LinkStateMjServer::LinkStateMjServer(mjModel *mj_model, std::string cfg_path):
    _m(mj_model)
{
    _ml = MatLogger2::MakeLogger("/tmp/mjlogger");

    auto ml = _ml;

    _ml->set_on_data_available_callback(
        [this](VariableBuffer::BufferInfo bi)
        {
            if(bi.variable_free_space < 0.5)
            {
                std::cout << "flushing to mat file \n";
                _ml->flush_available_data();
            }
        });

    for(int siteid = 0; siteid < mj_model->nsite; siteid++)
    {

        std::string sitename = &_m->names[mj_model->name_siteadr[siteid]];

        Hal::DeviceInfo dinfo;
        dinfo.id = -1;
        dinfo.name = sitename + "_lss";
        dinfo.type = "link_state_mj";

        auto lss = std::make_shared<LinkStateInstanceMj>(dinfo);
        lss->site_id = siteid;

        std::cout << "found site " << sitename << "\n";

        _lss.push_back(lss);

    }

    std::vector<Hal::DeviceRt::Ptr> devs(_lss.begin(), _lss.end());

    _srv = std::make_unique<ServerManager>(devs, "sock", "link_state_sensor");

    _J.resize(6, _m->nv);
}

void LinkStateMjServer::run(mjData * d)
{
    auto qvel = Eigen::VectorXd::Map(d->qvel, _m->nv);

    for(auto lss : _lss)
    {
        auto t = Eigen::Vector3d::Map(&d->site_xpos[lss->site_id*3]);
        auto R = Eigen::Matrix3d::Map(&d->site_xmat[lss->site_id*9]);
        auto quat = Eigen::Quaterniond(R.transpose());

        Eigen::Vector4d::Map(lss->rx().orientation.data()) = quat.coeffs();
        Eigen::Vector3d::Map(lss->rx().position.data()) = t;

        _J.setZero();
        mj_jacSite(_m, d, _J.data(), _J.data() + 3*_J.cols(), lss->site_id);

        Eigen::Matrix<double, 6, 1>::Map(lss->rx().twist.data()).noalias() = _J * qvel;

    }

    _srv->send();
    _srv->run();
}


