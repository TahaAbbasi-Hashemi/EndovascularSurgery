#pragma once

#include <iostream>
#include <boost/asio.hpp>


class Serial{
    private:
        double something;
        boost::asio::io_service io;
        boost::asio::serial_port port;
        //port = boost::asio::serial(ioservice, "/dev/ttyACM0").new();

    public:
        Serial();
        ~Serial();

        // Setting Functions
        void s_default();
        bool s_portName(std::string port);
        bool s_baudRate(int rate);
        bool s_parity(int parity);
        bool s_stopBit(int bits);
        bool s_charSize(int size);

        // Doing Functions
        void sendString(std::string sendData);
        
        // Getting Functions
        std::string g_data();
}

