/*
    See:
    https://aslze.github.io/asl-doc/index.html
    Another option, not covered here:
    https://aslze.github.io/asl-doc/classasl_1_1_socket_server.html

    Covered here: Socket Communication, Part 2/2: Client side

    How to use:
    1. Select flag udp depending on what you'd like to test: UDP or TCP
    2. If UDP: 
        Start client on a terminal: client awaits for packets on port
            ./asl_test_02
        Start server on another terminal: server starts delivering packets
            ./asl_test_01
    3. If TCP
        Start server on a terminal: server awaits for connection
            ./asl_test_01
        Start client on another terminal: connection is stablished, packets are sent by the server & read by the client
            ./asl_test_02
    
*/


#include <iostream>
#include <string>

#include "asl/defs.h" // infinity(), nan(), sqr(), rad2deg(), deg2rad(), random(), max(), min(), clamp()
// defs includes
// - asl/time.h: sleep(), now()
// - asl/atomic.h: AtomicCount
#include "asl/Array.h" // Array
#include "asl/String.h" // String
#include "asl/Var.h" // Var
#include "asl/JSON.h" // Json
#include "asl/Directory.h" // File System; It indludes File.h
#include "asl/CmdArgs.h" // Command Line Argument Paring
#include "asl/Socket.h" // Sockets

using namespace asl;

int main(int argc, char** argv) {

    bool udp = false;
    bool tcp = !udp;

    // 1. UDP Socket: Simple Basic Type Receiver
    if (udp) {
        String hostUDP = "localhost"; // network name or IP
        int portUDP = 53427; // random big value
        PacketSocket socketUDP;
        socketUDP.bind(hostUDP, portUDP); // in UDP, both server and client bind() and that's enough
        int *data = new int; // packet
        std::cout << "Waiting for UDP packets..." << std::endl;
        while (socketUDP.waitInput(5.0)) {
            // Receive packet
            socketUDP.readFrom(InetAddress(hostUDP, portUDP), (void*)data, sizeof(data));
            std::cout << "<- " << *data << std::endl;
        }
        std::cout << "Finished!" << std::endl;
    }

    // 2. TCP Socket: JSON string receiver/client
    if (tcp) {
        String hostTCP = "localhost"; // network name or IP
        int portTCP = 53529; // random big value
        Socket socketTCP;
        socketTCP.connect(hostTCP, portTCP); // servers bind() to a local port, clients connect() to a remote server
        // In TCP, servers need to connect(), and then read
        std::cout << "Waiting for TCP packets..." << std::endl;
        while (!socketTCP.disconnected()) {
            if (!socketTCP.waitData())
                continue;
            // Receive packet
            String line = socketTCP.readLine(); // server send strings with EOL (end-of-line)
            // Decode packet and display
            Var packet = Json::decode(line);
            if (packet.ok()) {
                printf("<- %s\n", *packet.toString());
            }
        }
        std::cout << "Finished!" << std::endl;
    }

}