#include "xbot2_mj_clock.h"

using namespace XBot;

ClockServer::ClockServer() {

    /* Create mq */
    using namespace std::chrono_literals;
    chrono::simulated_clock::enable_sim_time(true);

    chrono::simulated_clock::initialize(0s);

    _clock_sender = Transmitter::Create("mq",
                                        TR_NAME,
                                        10,
                                        sizeof(std::chrono::nanoseconds));

}

ClockServer::~ClockServer()
{

}

void ClockServer::reset(mjData * d) {
    mj_stime_ref = mj_stime;
    mj_stime = d->time;
}

void ClockServer::run(mjData * d) {
    using namespace std::chrono;
    mj_stime = d->time+mj_stime_ref;

    // Convert the double time to an integer representing nanoseconds
    mj_stime_ns = static_cast<int64_t>(mj_stime*1e9);

    // Compute seconds and nanoseconds
    int sec = static_cast<int>(mj_stime_ns / 1000000000);  // 1 second = 1e9 nanoseconds
    int nsec = static_cast<int>(mj_stime_ns % 1000000000);  // Remaining nanoseconds
    
    mj_stime_chrono = seconds(sec) + nanoseconds(nsec);
    chrono::simulated_clock::set_time(mj_stime_chrono);
    _clock_sender->try_send(mj_stime_chrono);
}