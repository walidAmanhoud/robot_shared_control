#include "FalconControl.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "falcon_control");
  ros::NodeHandle n;
  float frequency = 200.0f;
  
  FalconControl falconControl(n,frequency);

  if (!falconControl.init()) 
  {
    return -1;
  }
  else
  {
   
    falconControl.run();
  }

  return 0;
}

