#include "serialIO.hpp"
#include <vector>

using std::vector;

int main()
{
     SerialIO roomba("/dev/tty.usbserial-DN0267K3", 115200);
     roomba.writeBytes(vector<unsigned char>{0x8B, 0x04, 0x00, 0x80});

}
