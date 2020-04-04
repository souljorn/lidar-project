#include <rplidar.h>

const int pwmPin = 36;
const int rpm=600; 

//Use Teensy 3.5 hardware serial
RPLidar lidar(1, pwmPin); 
RPLidarPacket packet;

void setup()
{
  lidar.setup(rpm);
  lidar.start();
}
  
void loop()
{
  //this function doesn't block waiting for the packet
  //but it expects the same from the rest of your code
  if( lidar.processAvailable(&packet) )
  {
    //do something with the packet
  }
}
