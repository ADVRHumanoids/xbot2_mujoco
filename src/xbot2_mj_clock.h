#ifndef XBOT2_MJ_CLOCK
#define XBOT2_MJ_CLOCK

#include "xbot2/ipc/pipe.h"
#include "xbot2/system/clock.h"
#include <xbot2/client_server/server_manager.h>
#include <mujoco/mujoco.h>

namespace XBot
{

class ClockServer
{

public:

    XBOT2_DECLARE_SMART_PTR(ClockServer);

    ClockServer();
    ~ClockServer();

    void run(mjData * d);

    void reset(mjData * d);

private:

    ClockServer::UniquePtr _srv;

    Transmitter::Ptr _clock_sender;
    Receiver::Ptr _tmax_recv;

    std::string TR_NAME="gz_to_xbot2_time";
    
    double mj_stime=0;
    double mj_stime_ref=0;
    int64_t mj_stime_ns=0;

    std::chrono::nanoseconds mj_stime_chrono;

};
    
}

#endif // XBOT2_MJ_CLOCK
