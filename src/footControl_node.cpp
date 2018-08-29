#include "FootControl.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "foot_control");
  ros::NodeHandle n;
  float frequency = 200.0f;
  
  FootControl footControl(n,frequency);

  if (!footControl.init()) 
  {
    return -1;
  }
  else
  {
   
    footControl.run();
  }

  return 0;
}

