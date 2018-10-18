#include "FalconControl.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "falcon_control");
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

  FalconControl falconControl(n,frequency,filename);

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

