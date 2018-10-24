#ifndef ROOMBA_HPP 
#define ROOMBA_HPP 

#include "../include/serialIO.hpp"
#include <vector>
#include <cmath>
#include <utility>

class Roomba{
public:
    Roomba(std::string port, unsigned int baudRate);
    void start();
    void stop();
    void setForward(short velocity);
    void turn(short velocity);
    void update(float timeStep);
    std::pair<float,float> getPosition();
    float getDirection();
    int getForwardVelocity();
    int getTurningVelocity();
    int BumperAndWheels();
    int WheelOverCurruent();
private: 
    const float PI = 3.14159265359;
    SerialIO serialPort;
    int forwardVelocity;
    int turningVelocity;
    std::pair<float,float> position;
    float direction;
    float radius;
};

#endif
