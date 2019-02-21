#include "FeetTelemanipulation.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "feet_telemanipulation");
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

  FeetTelemanipulation feetTelemanipulation(n,frequency,filename);

  if (!feetTelemanipulation.init()) 
  {
    return -1;
  }
  else
  {
   
    feetTelemanipulation.run();
  }

  return 0;
}

