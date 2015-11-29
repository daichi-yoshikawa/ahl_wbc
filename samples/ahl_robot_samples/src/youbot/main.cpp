#include "ahl_utils/exception.hpp"
#include "ahl_robot_samples/youbot/youbot.hpp"

using namespace ahl_sample;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "youbot_sample");
  ros::NodeHandle nh;

  try
  {
    YouBotPtr youbot = YouBotPtr(new YouBot());
    youbot->init();
    youbot->run();
  }
  catch(ahl_utils::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return -1;
  }
  catch(ahl_gazebo_if::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return -1;
  }
  catch(...)
  {
    ROS_ERROR_STREAM("Unknown exception occurred.");
    return -1;
  }

  return 0;
}
