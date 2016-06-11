#include "ahl_utils/exception.hpp"
#include "ahl_robot_samples/red_arm/red_arm.hpp"

using namespace ahl_sample;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "red_arm_sample");
  ros::NodeHandle nh;

  try
  {
    RedArmPtr red_arm = std::make_shared<RedArm>();
    red_arm->init();
    red_arm->run();
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
}
