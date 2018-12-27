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

        unsigned char txrx(unsigned char address, unsigned char command, unsigned char *tx_data, unsigned int tx_length,
                           unsigned char *rx_data, unsigned int rx_length, bool tx_crc = false, bool rx_crc = false);


        void timeout_handler();

        void read_callback();

    };

    class timeout_exception : public std::runtime_error {
    public:
        using std::runtime_error::runtime_error;
    };

}
#endif //PROJECT_ROBOCLAWDRIVER_H
