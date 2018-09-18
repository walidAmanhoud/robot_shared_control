#include "FootControl.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "foot_control");
  ros::NodeHandle n;
  float frequency = 200.0f;
  std::string filename;
  if(argc==2)
  {
    filename = std::string(argv[1]);
  }
  else
  {
    return -1;
  }

  FootControl footControl(n,frequency,filename);

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

