#ifndef MJLSS_H
#define MJLSS_H

#include <xbot2/hal/dev_imu.h>
#include <xbot2/client_server/server_manager.h>

#include <mujoco/mujoco.h>

#include <matlogger2/matlogger2.h>

namespace XBot
{

struct lss_rx
{
    std::array<double, 3> position;
    std::array<double, 4> orientation;
    std::array<double, 6> twist;
};

struct lss_tx
{

};



class LinkStateInstanceMj : public Hal::DeviceTplCommon<lss_rx, lss_tx>
{

public:

    XBOT2_DECLARE_SMART_PTR(LinkStateInstanceMj)

    typedef DeviceTplCommon<lss_rx, lss_tx> BaseType;

    LinkStateInstanceMj(Hal::DeviceInfo devinfo);

    int site_id;


    // DeviceBase interface
public:
    bool sense() override {return true;}
    bool move() override {return true;}
};

class LinkStateMjServer
{

public:

    XBOT2_DECLARE_SMART_PTR(LinkStateMjServer);

    LinkStateMjServer(mjModel * mj_model, std::string cfg_path);

    void run(mjData * d);

private:

    ServerManager::UniquePtr _srv;
    std::vector<LinkStateInstanceMj::Ptr> _lss;
    mjModel * _m;
    Eigen::Matrix<double, 6, -1, Eigen::RowMajor> _J;

    MatLogger2::Ptr _ml;

};

}

#endif // MJIMU_H
