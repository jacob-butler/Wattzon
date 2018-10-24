#include "../include/roomba.hpp"
#include <vector>
#include <iostream>

using std::vector;
using std::cout;
using std::endl;

int main()
{
     SerialIO roomba("/dev/tty.usbserial-DN0267K3", 115200);
     while(true){
         roomba.writeBytes(vector<unsigned char>{149,1,7});
         cout << (int)roomba.readBytes(1)[0] << endl;
     }
}
