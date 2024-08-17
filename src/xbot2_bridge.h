#ifndef XBOT2_BRIDGE_H
#define XBOT2_BRIDGE_H

#include "xbot2_mj_imu.h"
#include "xbot2_mj_joint.h"
#include "xbot2_mj_linkstate.h"

namespace XBot {

class MjWrapper
{

public:

    XBOT2_DECLARE_SMART_PTR(MjWrapper)

    MjWrapper(mjModel * mj_model, std::string cfg_path):
        _joint(mj_model, cfg_path),
        _imu(mj_model, cfg_path),
        _ls(mj_model, cfg_path)
    {
        fprintf(stderr, "[XBot][MjWrapper]: xbot2 bridge constructed with file  %s \n",cfg_path.c_str());
    }

    void run(mjData * d)
    {
        _joint.run(d);
        _imu.run(d);
        _ls.run(d);
    }

    void move_to_homing_now(mjData * d) {
        _joint.run(d);
    }

private:

    JointMjServer _joint;
    ImuMjServer _imu;
    LinkStateMjServer _ls;

};

}


#endif // XBOT2_BRIDGE_H
