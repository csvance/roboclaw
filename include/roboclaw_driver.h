//
// Created by Carroll Vance on 2018-12-26.
//

#ifndef PROJECT_ROBOCLAWDRIVER_H
#define PROJECT_ROBOCLAWDRIVER_H

#include <string>
#include <map>
#include <exception>

#include <boost/asio.hpp>

namespace roboclaw {

    class driver {

    public:
        driver(std::string &port);

        driver(std::string &port, std::shared_ptr<std::map<int, unsigned char>> address_map);

        ~driver();

        void set_baud(unsigned int baudrate);

        static unsigned char BASE_ADDRESS;
        static unsigned int DEFAULT_BAUDRATE;

    private:
        std::shared_ptr<std::map<int, unsigned char>> address_map;
        std::shared_ptr<boost::asio::serial_port> serial;
        boost::asio::io_service io;

        void init_serial(std::string &port, unsigned int baudrate = DEFAULT_BAUDRATE);

        static unsigned int crc16(unsigned char *packet, int nBytes);

        unsigned char cmd8(unsigned char address, unsigned char command, unsigned char data, bool append_crc = false);


        void timeout_handler();

        void read_callback();

    };

    class timeout_exception : public std::runtime_error {
    public:
        using std::runtime_error::runtime_error;
    };

}
#endif //PROJECT_ROBOCLAWDRIVER_H
