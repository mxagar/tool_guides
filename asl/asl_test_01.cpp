/*
    This example shows how to implement UDP and TCP servers for communication using ASL.
    The UDP server sends an int to the client.
    Th TCP server sends a JSON string with an EOL (\n) to the client.

    See:
    https://aslze.github.io/asl-doc/index.html
    Another option, not covered here:
    https://aslze.github.io/asl-doc/classasl_1_1_socket_server.html

    Covered here: Socket Communication, Part 1/2: Server side

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

    // 1. UDP Socket: Simple Basic Type Sender
    if (udp) {
        String hostUDP = "localhost"; // network name or IP
        int portUDP = 53427; // random big value
        PacketSocket socketUDP;
        socketUDP.bind(hostUDP, portUDP); // in UDP, both server and client bind() and that's enough
        int *data = new int; // packet
        unsigned long int maxPacketNumber = 1000;
        std::cout << "Sending UDP packets..." << std::endl;
        for (unsigned int i = 0; i < maxPacketNumber; ++i) {
            // Update packet and send
            *data = i;
            socketUDP.sendTo(InetAddress(hostUDP, portUDP), (void*)data, sizeof(data));
            std::cout << "-> " << i << std::endl;
        }
        std::cout << "Finished!" << std::endl;
    }

    if (tcp) {
        // 2. JSON Object: Measurement
        Var measurement = Var("name", "measurement")
                        ("id", 73)
                        ("reference", 42)
                        ("time", Date::now())
                        ("successful", true)
                        ("values", Var::array({1.1f, 2.2f, 3.3f, 4.4f, 5.5f, 6.6f}));

        String measurement_json = Json::encode(measurement); // Encode Var as JSON string; equivalent to JavaScript stringify()
        bool ret_json = Json::write("measurement.json", measurement); // Writes a var to a file in JSON format
        Var measurement_read = Json::read("measurement.json"); // Read file data.json which is in JSON format, ans save it as Var
        if (measurement_read.ok())
            printf("%s\n", *measurement_read.toString()); // Print read JSON file

        // 3. TCP Socket: JSON string sender/server
        String hostTCP = "localhost"; // network name or IP
        int portTCP = 53529; // random big value
        Socket socketConnection; // first socket which waits for connect() from client
        socketConnection.bind(hostTCP, portTCP); // servers bind() to a local port, clients connect() to a remote server
        // In TCP, servers need to bind(), listen() & accept()
        // The socket returned by accept() is used to communicate/send/write
        socketConnection.listen();
        Socket socketTCP = socketConnection.accept();
        unsigned long int maxPacketNumber = 1000;
        std::cout << "Sending TCP packets..." << std::endl;
        for (unsigned int i = 0; i < maxPacketNumber; ++i) {
            // Update packet
            measurement["id"] = i;
            measurement["time"] = Date::now();
            measurement["values"] =  Var::array({float(i), float(i*2), float(i*3), float(i*4), float(i*5), float(i*6)});
            // Prepare packet and send
            String packet = Json::encode(measurement);
            socketTCP << packet + "\n"; // pass string with EOL (end-of-line), then readLine in the client
            std::cout << "-> [" << packet.length() << "]: " << std::string(packet) << std::endl;
        }
        std::cout << "Finished!" << std::endl;
    }
   

}