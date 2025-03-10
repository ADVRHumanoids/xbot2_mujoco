#ifndef XBOT2_BRIDGE_H
#define XBOT2_BRIDGE_H

#include "xbot2_mj_imu.h"
#include "xbot2_mj_joint.h"
#include "xbot2_mj_linkstate.h"

#include <xbot2/ipc/pipe.h>
#include <xbot2/executor/xbot2executor.h>

#include <thread>

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
        auto do_run = [this, d](){
            _joint.run(d);
            _imu.run(d);
            _ls.run(d);

            using ns = std::chrono::nanoseconds;
            ns clock_msg = std::chrono::duration_cast<ns>(std::chrono::duration<double>(d->time));
            _clock_sender->try_send(clock_msg);
        };
        
        if(!_xbot2){

            std::atomic_bool xbot2_init_in_progress(true);

            std::thread t([&xbot2_init_in_progress, &do_run](){
                while(xbot2_init_in_progress)
                {
                    do_run();
                    this_thread::sleep_for(10ms);
                }
            });
            
            _xbot2 = std::make_unique<XBot2Executor>(
                "/home/iit.local/alaurenzi/code/ros2_ws/src/iit-centauro-ros-pkg/centauro_config/centauro_basic.yaml",
                "sim", 
                true,
                true, 
                false
            );


            xbot2_init_in_progress = false;

            t.join();
        }

        // say we're at iter k

        // send telemetry to xbot (t[k], y[k] -> this also contains u[k-1])
        // recv commands from xbot u[k]
        // set commands
        do_run();

        // recv telemetry from sim (t[k], y[k])
        // run user plugins
        // send commands to sim for next iteration u[k+1] = g(y[k], t[k])
        _xbot2->run(false, true);

        // advance state to x[k+1] = f(x[k], u[k])

        
    }
private:

    JointMjServer _joint;
    ImuMjServer _imu;
    LinkStateMjServer _ls;
    std::unique_ptr<XBot2Executor> _xbot2;

    Transmitter::Ptr _clock_sender;

};

}


#endif // XBOT2_BRIDGE_H
