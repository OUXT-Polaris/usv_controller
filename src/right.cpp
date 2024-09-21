#include <usv_controller/thruster_command.pb.h>
#include <protolink/client.hpp>
#include <chrono>

int main()
{
    boost::asio::io_service io;
    protolink::udp_protocol::Publisher pub(io, "192.168.0.211", 2000, 2001);
    double thrust = -0.3;
    try {
        while(true) {
            // thrust = 0;
            communication::Command command;
            command.set_thrust(thrust);
            command.set_emergency_stop(false);
            pub.send(command);
            std::cout << "RIGHT:" << thrust << std::endl;
            thrust = thrust + 0.1;
            if(thrust >= 0.31) {
                thrust = -0.3;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    catch(...) {
        communication::Command command;
        command.set_thrust(0);
        command.set_emergency_stop(false);
        pub.send(command);
    }
}
