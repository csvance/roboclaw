/**
 *
 * Copyright (c) 2018 Carroll Vance.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "roboclaw_driver.h"

#include <vector>
#include <boost/bind.hpp>
#include <boost/array.hpp>

namespace roboclaw {

    unsigned char driver::BASE_ADDRESS = 128;
    unsigned int driver::DEFAULT_BAUDRATE = 115200;

    driver::driver(std::string &port) {
        init_serial(port);
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

    unsigned int driver::crc16(unsigned char *packet, size_t nBytes) {
        unsigned int crc = 0;

        for (size_t byte = 0; byte < nBytes; byte++) {
            crc = crc ^ ((unsigned int) packet[byte] << 8);
            for (unsigned char bit = 0; bit < 8; bit++) {
                if (crc & 0x8000)
                    crc = (crc << 1) ^ 0x1021;
            }
        }
        crc = crc << 1;

        return crc;
    }

    size_t driver::txrx(unsigned char address,
                            unsigned char command,
                            unsigned char *tx_data,
                            size_t tx_length,
                            unsigned char *rx_data,
                            size_t rx_length,
                            bool tx_crc, bool rx_crc) {

        std::vector<unsigned char> packet;

        if (tx_crc)
            packet.reserve(tx_length + 4);
        else
            packet.reserve(tx_length + 2);

        // Header
        packet[0] = address;
        packet[1] = command;

        // Data
        if(tx_length > 0 && tx_data != nullptr)
            memcpy(&packet[2], tx_data, tx_length);

        // CRC
        if (tx_crc) {
            unsigned int crc = crc16(&packet[0], tx_length + 2);

            // RoboClaw expects big endian / MSB first
            packet[tx_length + 2] = (unsigned char) ((crc >> 8) & 0xFF);
            packet[tx_length + 2 + 1] = (unsigned char) (crc & 0xFF);

        }

        serial->write_some(boost::asio::buffer(packet));

        std::vector<unsigned char> response;
        if (rx_crc)
            response.reserve(rx_length + 2);
        else
            response.reserve(rx_length);

        bool timeout = false;
        bool error = false;

        size_t bytes_received = 0;

        boost::system::error_code error_code;

        boost::asio::deadline_timer timer(io);
        timer.expires_from_now(boost::posix_time::milliseconds(100));

        timer.async_wait([timeout](const boost::system::error_code &ec) mutable { timeout = true; });

        serial->async_read_some(boost::asio::buffer(response),
                                [error, error_code, bytes_received](const boost::system::error_code &ec,
                                                                    std::size_t bytes_transferred) mutable {

                                    if (bytes_transferred == 0)
                                        error = true;

                                    error_code = ec;
                                    bytes_received = bytes_transferred;
                                });

        io.reset();
        io.run_one();


        if (error)
            throw timeout_exception("Roboclaw read timed out");
        else if (timeout)
            throw std::runtime_error(error_code.message());

        // Copy response back
        memcpy(rx_data, &response[0], bytes_received);

        // Check CRC
        if (rx_crc) {
            unsigned int crc_calculated = crc16(&response[0], bytes_received - 2);
            unsigned int crc_received = 0;

            // RoboClaw generates big endian / MSB first
            crc_received += response[bytes_received - 2] << 8;
            crc_received += response[bytes_received - 1];

            if (crc_calculated != crc_received)
                throw std::runtime_error("Roboclaw CRC mismatch");

        }

        if (!rx_crc)
            return bytes_received;
        else
            return bytes_received - 2;
    }

    std::string driver::get_version(unsigned char address) {

        unsigned char rx_buffer[48];

        txrx(address, 21, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        std::string version = std::string( reinterpret_cast< char const* >(rx_buffer) );
        trim(version);

        return version;

    }

    std::pair<int, int> driver::get_encoders(unsigned char address){

        unsigned char rx_buffer[7];

        txrx(address, 16, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        uint32_t e1 = 0;

        e1 += rx_buffer[0] << 24;
        e1 += rx_buffer[1] << 16;
        e1 += rx_buffer[2] << 8;
        e1 += rx_buffer[3];

        txrx(address, 17, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        uint32_t e2 = 0;

        e2 += rx_buffer[0] << 24;
        e2 += rx_buffer[1] << 16;
        e2 += rx_buffer[2] << 8;
        e2 += rx_buffer[3];

        return std::pair<int,int>((int)(int32_t)e1, (int)(int32_t)e2);
    }

    std::pair<int, int> driver::get_velocity(unsigned char address){

        unsigned char rx_buffer[7];

        txrx(address, 18, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        uint32_t e1 = 0;

        e1 += rx_buffer[0] << 24;
        e1 += rx_buffer[1] << 16;
        e1 += rx_buffer[2] << 8;
        e1 += rx_buffer[3];

        txrx(address, 19, nullptr, 0, rx_buffer, sizeof(rx_buffer), false, true);

        uint32_t e2 = 0;

        e2 += rx_buffer[0] << 24;
        e2 += rx_buffer[1] << 16;
        e2 += rx_buffer[2] << 8;
        e2 += rx_buffer[3];

        return std::pair<int,int>((int)(int32_t)e1, (int)(int32_t)e2);
    }

    void driver::reset_encoders(unsigned char address) {
        unsigned char rx_buffer[1];
        txrx(address, 20, nullptr, 0, rx_buffer, sizeof(rx_buffer), true, false);
    }

    void driver::set_speed(unsigned char address, std::pair<int, int> speed) {
        unsigned char rx_buffer[1];
        unsigned char tx_buffer[8];

        tx_buffer[0] = (unsigned char)((speed.first >> 24) & 0xFF);
        tx_buffer[1] = (unsigned char)((speed.first >> 16) & 0xFF);
        tx_buffer[2] = (unsigned char)((speed.first >> 8) & 0xFF);
        tx_buffer[3] = (unsigned char)((speed.first) & 0xFF);

        tx_buffer[4] = (unsigned char)((speed.second >> 24) & 0xFF);
        tx_buffer[5] = (unsigned char)((speed.second >> 16) & 0xFF);
        tx_buffer[6] = (unsigned char)((speed.second >> 8) & 0xFF);
        tx_buffer[7] = (unsigned char)((speed.second) & 0xFF);

        txrx(address, 37, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer), true, false);
    }

}