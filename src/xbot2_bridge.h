#ifndef XBOT2_BRIDGE_H
#define XBOT2_BRIDGE_H

#include "xbot2_mj_imu.h"
#include "xbot2_mj_joint.h"

namespace XBot {

class MjWrapper
{

public:

    XBOT2_DECLARE_SMART_PTR(MjWrapper)

    MjWrapper(mjModel * mj_model, std::string cfg_path):
        _joint(mj_model, cfg_path),
        _imu(mj_model, cfg_path)
    {

    }

    void run(mjData * d)
    {
        _joint.run(d);
        _imu.run(d);
    }

private:

    JointMjServer _joint;
    ImuMjServer _imu;


};

}


#endif // XBOT2_BRIDGE_H
