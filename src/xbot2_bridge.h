#ifndef XBOT2_BRIDGE_H
#define XBOT2_BRIDGE_H

#include "xbot2_mj_imu.h"
#include "xbot2_mj_joint.h"
#include "xbot2_mj_linkstate.h"

#include <xbot2/ipc/pipe.h>

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
        std::cout << "************ xbot2 bridge constructed with file: " << cfg_path << "\n";

        /* Create mq for clock */
        using namespace std::chrono_literals;
        chrono::simulated_clock::enable_sim_time(true);
        chrono::simulated_clock::initialize(0s);

        _clock_sender = Transmitter::Create("mq",
                                            "gz_to_xbot2_time",
                                            10,
                                            sizeof(std::chrono::nanoseconds));
    }

    void run(mjData * d)
    {
        _joint.run(d);
        _imu.run(d);
        _ls.run(d);

        using ns = std::chrono::nanoseconds;
        ns clock_msg = std::chrono::duration_cast<ns>(std::chrono::duration<double>(d->time));
        _clock_sender->try_send(clock_msg);
    }
private:

    JointMjServer _joint;
    ImuMjServer _imu;
    LinkStateMjServer _ls;

    Transmitter::Ptr _clock_sender;

};

}


#endif // XBOT2_BRIDGE_H
