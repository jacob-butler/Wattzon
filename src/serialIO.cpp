#include "../include/serialIO.hpp"

using std::vector;
using std::size_t;

SerialIO::SerialIO(std::string port, unsigned int baudRate)
: io(), serial(io,port){
    serial.set_option(boost::asio::serial_port_base::baud_rate(baudRate)); 
}

void SerialIO::writeBytes(const vector<unsigned char> & commands){
    boost::system::error_code ec;
    boost::asio::write(serial, boost::asio::buffer(commands), ec);
}
vector<unsigned char> SerialIO::readBytes(size_t size){
    vector<unsigned char> data(size);
    boost::system::error_code ec;
    boost::asio::read(serial, boost::asio::buffer(data), ec);
    return data;
}

