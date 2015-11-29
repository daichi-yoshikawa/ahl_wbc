#include "ahl_utils/exception.hpp"
#include "ahl_robot_samples/pr2/pr2.hpp"

using namespace ahl_sample;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_sample");
  ros::NodeHandle nh;

  try
  {
    PR2Ptr pr2 = PR2Ptr(new PR2());
    pr2->init();
    pr2->run();
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
