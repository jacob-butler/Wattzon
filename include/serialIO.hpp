#ifndef SERIALIO_HPP
#define SERIALIO_HPP

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>
#include <vector>
#include <cstdlib>

class SerialIO {
public:
    SerialIO(std::string port, unsigned int baudRate);
    void writeBytes(const std::vector<unsigned char> & commands);
    std::vector<unsigned char> readBytes(std::size_t size);
private:
    boost::asio::io_service io;
    boost::asio::serial_port serial;     
};

#endif
