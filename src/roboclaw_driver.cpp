//
// Created by Carroll Vance on 2018-12-26.
//

#include "../include/roboclaw_driver.h"

#include <boost/bind.hpp>
#include <boost/array.hpp>

namespace roboclaw {

    unsigned char driver::BASE_ADDRESS = 128;
    unsigned int driver::DEFAULT_BAUDRATE = 115200;

    driver::driver(std::string &port) {
        init_serial(port);

        this->address_map = std::shared_ptr<std::map<int, unsigned char>>(new std::map<int, unsigned char>());
        this->address_map->insert(std::pair<int, unsigned char>(0, BASE_ADDRESS));
    }

    driver::driver(std::string &port, std::shared_ptr<std::map<int, unsigned char>> address_map) {
        init_serial(port);

        this->address_map = address_map;
    }

    driver::~driver() {
        this->serial->close();
    }

    void driver::init_serial(std::string &port, unsigned int baudrate) {
        serial = std::shared_ptr<boost::asio::serial_port>(new boost::asio::serial_port(io, port));
        set_baud(baudrate);
    }

    void driver::set_baud(unsigned int baudrate) {
        this->serial->set_option(boost::asio::serial_port_base::baud_rate(baudrate));
    }

    unsigned int driver::crc16(unsigned char *packet, int nBytes) {
        unsigned int crc = 0;

        for (int byte = 0; byte < nBytes; byte++) {
            crc = crc ^ ((unsigned int) packet[byte] << 8);
            for (unsigned char bit = 0; bit < 8; bit++) {
                if (crc & 0x8000)
                    crc = (crc << 1) ^ 0x1021;
            }
        }
        crc = crc << 1;

        return crc;
    }

    unsigned char driver::cmd8(unsigned char address, unsigned char command, unsigned char data, bool append_crc) {

        if(append_crc) {
            boost::array<unsigned char, 5> packet = {address, command, data, 0, 0};

            unsigned int crc = crc16(&packet[0], 3);

            // RoboClaw expects big endian / MSB first
            packet[3] = (unsigned char) ((crc >> 8) & 0xFF);
            packet[4] = (unsigned char) (crc & 0xFF);

            serial->write_some(boost::asio::buffer(packet));
        }else {
            boost::array<unsigned char, 3> packet = {address, command, data};

            serial->write_some(boost::asio::buffer(packet));
        }

        boost::array<unsigned char, 1> response = {0};

        bool timeout = false;
        bool error = false;

        boost::system::error_code error_code;

        boost::asio::deadline_timer timer(io);
        timer.expires_from_now(boost::posix_time::milliseconds(100));

        timer.async_wait([timeout](const boost::system::error_code &ec) mutable { timeout = true; });

        serial->async_read_some(boost::asio::buffer(response),
                                [error, error_code](const boost::system::error_code &ec,
                                                    std::size_t bytes_transferred) mutable {
                                    error = true;
                                    error_code = ec;
                                });

        io.reset();
        io.run_one();

        if (timeout) {
            if (!error)
                throw timeout_exception("Roboclaw read timed out");
            else
                throw std::runtime_error(error_code.message());
        }

        return response[0];
    }

}