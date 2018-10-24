#include "../include/roomba.hpp"
#include <iostream>

using std::vector;
using std::pair;

void wait(float numberOfNanoSecond){
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> diff = end-start;
    while(true){
        end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> diff = end-start;
        if(diff.count()*1000 > numberOfNanoSecond) return;
    }
    return; 
}

Roomba::Roomba(std::string port, unsigned int baudRate)
    :serialPort(port, baudRate),forwardVelocity(0),turningVelocity(0),position(0,0),radius(105){}

void Roomba::start(){
    serialPort.writeBytes(vector<unsigned char>{128});
    serialPort.writeBytes(vector<unsigned char>{132});
}

void Roomba::stop(){
    serialPort.writeBytes(vector<unsigned char>{173});
}

void Roomba::setForward(short velocity){
    forwardVelocity = (float)velocity; 
    turningVelocity = 0; 
    unsigned char leftHighByte = (velocity&0xFF00)>>8;
    unsigned char leftLowByte = (velocity&0x00FF);
    unsigned char rightHighByte = ((velocity)&0xFF00)>>8;
    unsigned char rightLowByte = ((velocity)&0x00FF);

    serialPort.writeBytes(vector<unsigned char>{145});
    wait(10);
    serialPort.writeBytes(vector<unsigned char>{rightHighByte, rightLowByte, leftHighByte, leftLowByte});
}
void Roomba::turn(short velocity){
    turningVelocity = (float)velocity; 
    forwardVelocity = 0;
    unsigned char highByteRight = (velocity&0xFF00)>>8;
    unsigned char lowByteRight = (velocity&0x00FF);
    unsigned char highByteLeft = ((-velocity)&0xFF00)>>8;
    unsigned char lowByteLeft = ((-velocity)&0x00FF);
    serialPort.writeBytes(vector<unsigned char>{145});
    wait(10);
    serialPort.writeBytes(vector<unsigned char>{highByteRight, lowByteRight, highByteLeft, lowByteLeft});
}

void Roomba::update(float timeStep){ 
    float angularVelocity = 180/PI*turningVelocity/radius;
    direction += angularVelocity*timeStep;
    position.first += std::cos(direction*PI/180)*forwardVelocity*timeStep;
    position.second += std::sin(direction*PI/180)*forwardVelocity*timeStep;  
}

pair<float,float> Roomba::getPosition(){
    return position;
}
float Roomba::getDirection(){
    return direction;
}
int Roomba::getForwardVelocity(){
     return forwardVelocity;
}
int Roomba::getTurningVelocity(){
     return turningVelocity;
}


int Roomba::BumperAndWheels(){
    serialPort.writeBytes(vector<unsigned char>{149,1,7});
    return (int)serialPort.readBytes(1)[0];
}
int Roomba::WheelOverCurruent(){
    serialPort.writeBytes(vector<unsigned char>{149,1,14});
    return (int)serialPort.readBytes(1)[0];
}




