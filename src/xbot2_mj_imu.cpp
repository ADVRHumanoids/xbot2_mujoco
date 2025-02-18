#include "xbot2_mj_imu.h"

using namespace XBot;

ImuInstanceMj::ImuInstanceMj(Hal::DeviceInfo devinfo):
    BaseType(devinfo)
{

}

ImuMjServer::ImuMjServer(mjModel *mj_model, std::string cfg_path):
    _m(mj_model)
{
    struct ImuParams
    {
        int gyro_id;
        int acc_id;
        int site_id;
    };

    std::map<std::string, ImuParams> site_imu_map;

    for(int i = 0; i < mj_model->nsensor; i++)
    {
        std::string sensorname = &_m->names[mj_model->name_sensoradr[i]];

        std::cout << "found sensor " << sensorname << "\n";

        if(_m->sensor_objtype[i] != mjtObj::mjOBJ_SITE)
        {
            continue;
        }

        int siteid = _m->sensor_objid[i];
        std::string sitename = &_m->names[mj_model->name_siteadr[siteid]];

        if(_m->sensor_type[i] == mjtSensor::mjSENS_GYRO)
        {
            std::cout << "gyro found: " << sensorname << ", site " << sitename << "\n";
            site_imu_map[sitename].gyro_id = i;
            site_imu_map[sitename].site_id = siteid;
        }

        if(_m->sensor_type[i] == mjtSensor::mjSENS_ACCELEROMETER)
        {
            std::cout << "accelerometer found: " << sensorname << ", site " << sitename << "\n";
            site_imu_map[sitename].acc_id = i;
            site_imu_map[sitename].site_id = siteid;
        }
    }

    for(auto item : site_imu_map)
    {
        Hal::DeviceInfo di;
        di.name = item.first;
        di.type = "imu_mj";
        di.id = -1;

        auto imu = std::make_shared<ImuInstanceMj>(di);
        imu->gyro_id = item.second.gyro_id;
        imu->acc_id = item.second.acc_id;
        imu->site_id = item.second.site_id;

        _imus.push_back(imu);
    }

    std::vector<Hal::DeviceRt::Ptr> devs(_imus.begin(), _imus.end());

    _srv = std::make_unique<ServerManager>(devs, "sock", "imu_gz");
}

void ImuMjServer::run(mjData * d)
{
    for(auto imu : _imus)
    {
        int gyro_adr = _m->sensor_adr[imu->gyro_id];
        imu->rx().ang_vel_x = d->sensordata[gyro_adr];
        imu->rx().ang_vel_y = d->sensordata[gyro_adr+1];
        imu->rx().ang_vel_z = d->sensordata[gyro_adr+2];

        int acc_adr = _m->sensor_adr[imu->acc_id];
        imu->rx().lin_acc_x = d->sensordata[acc_adr];
        imu->rx().lin_acc_y = d->sensordata[acc_adr+1];
        imu->rx().lin_acc_z = d->sensordata[acc_adr+2];

        auto R = Eigen::Matrix3d::Map(&d->site_xmat[9*imu->site_id]);
        auto quat = Eigen::Quaterniond(R);
        imu->rx().orientation_x = quat.x();
        imu->rx().orientation_y = quat.y();
        imu->rx().orientation_z = quat.z();
        imu->rx().orientation_w = quat.w();
    }

    _srv->send();
    _srv->run();
}


bool XBot::ImuInstanceMj::sense()
{
    return true;
}

bool XBot::ImuInstanceMj::move()
{
    return true;
}
