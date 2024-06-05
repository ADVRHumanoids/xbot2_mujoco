#ifndef MJIMU_H
#define MJIMU_H

#include <xbot2/hal/dev_imu.h>
#include <xbot2/client_server/server_manager.h>
#include <mujoco/mujoco.h>

namespace XBot
{

struct imu_rx
{
    float ang_vel_x;
    float ang_vel_y;
    float ang_vel_z;

    float lin_acc_x;
    float lin_acc_y;
    float lin_acc_z;

    float orientation_x;
    float orientation_y;
    float orientation_z;
    float orientation_w;
};

struct imu_tx
{

};


class ImuInstanceMj : public Hal::DeviceTplCommon<imu_rx, imu_tx>
{

public:

    XBOT2_DECLARE_SMART_PTR(ImuInstanceMj)

    typedef DeviceTplCommon<imu_rx, imu_tx> BaseType;

    ImuInstanceMj(Hal::DeviceInfo devinfo);

    int gyro_id;

    int acc_id;

    int site_id;


    // DeviceBase interface
public:
    bool sense() override;
    bool move() override;
};

class ImuMjServer
{

public:

    XBOT2_DECLARE_SMART_PTR(ImuMjServer);

    ImuMjServer(mjModel * mj_model, std::string cfg_path);

    void run(mjData * d);

private:

    ServerManager::UniquePtr _srv;
    std::vector<ImuInstanceMj::Ptr> _imus;
    mjModel * _m;

};

}

#endif // MJIMU_H
