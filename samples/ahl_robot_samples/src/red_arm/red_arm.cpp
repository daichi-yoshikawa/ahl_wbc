#include "ahl_utils/scoped_lock.hpp"
#include "ahl_utils/exception.hpp"
#include "ahl_robot_samples/red_arm/red_arm.hpp"

using namespace ahl_sample;

void RedArm::init()
{
  ros::NodeHandle local_nh("~");
  std::string yaml = "";

  local_nh.param<std::string>("robot_config", yaml, "");

  initRobot("red_arm", yaml);

  controller_ = std::make_shared<RobotController>();
  controller_->init(robot_);

  param_ = std::make_shared<RedArmParam>();

  ManipulatorPtr mnp = robot_->getManipulator("mnp");

  gravity_compensation_ = std::make_shared<GravityCompensation>(robot_);
  joint_control_ = std::make_shared<JointControl>(mnp);
  position_control_ = std::make_shared<PositionControl>(mnp, "gripper");
  orientation_control_ = std::make_shared<OrientationControl>(mnp, "gripper");

  joint_control_->setGoal(param_->q);
  position_control_->setGoal(param_->x);
  orientation_control_->setGoal(param_->R);

  controller_->addTask(joint_control_, 0);
  controller_->addTask(orientation_control_, 10);
  controller_->addTask(position_control_, 20);
  controller_->addTask(gravity_compensation_, 20);

  gazebo_interface_ = std::make_shared<GazeboInterface>();
  gazebo_interface_->addJoint("red_arm::joint1");
  gazebo_interface_->addJoint("red_arm::joint2");
  gazebo_interface_->addJoint("red_arm::joint3");
  gazebo_interface_->addJoint("red_arm::joint4");
  gazebo_interface_->addJoint("red_arm::joint5");
  gazebo_interface_->addJoint("red_arm::joint6");
  gazebo_interface_->addJoint("red_arm::joint7");
  gazebo_interface_->connect();

  tf_pub_ = std::make_shared<TfPublisher>();

  markers_ = std::make_shared<Markers>();
  markers_->add(std::make_shared<Marker>("gripper_target", "world"));
  markers_->setColor(0, 1, 0, 0.5);
  markers_->setScale(0.1);
}

void RedArm::run()
{
  ros::NodeHandle nh;

  timer_update_model_ = nh.createTimer(ros::Duration(0.01), &RedArm::updateModel, this);
  timer_control_ = nh.createTimer(ros::Duration(0.001), &RedArm::control, this);

  ros::MultiThreadedSpinner spinner;
  spinner.spin();
}

void RedArm::updateModel(const ros::TimerEvent&)
{
  try
  {
    ahl_utils::ScopedLock lock(mutex_);
    if(!joint_updated_) return;

    const double amp = 0.1;
    const double f = 0.3;
    Eigen::Vector3d dx = Eigen::Vector3d::Zero();
    static double time = 0.0;

    if(param_->sin_x)
      dx[0] = amp * sin(2.0 * M_PI * f * time);
    if(param_->sin_y)
      dx[1] = amp * cos(2.0 * M_PI * f * time);
    if(param_->sin_z)
      dx[2] = amp * sin(2.0 * M_PI * f * time);

    time += 0.01;

    position_control_->setGoal(param_->x + dx);

    robot_->computeJacobian();
    robot_->computeMassMatrix();
    controller_->updateModel();

    model_updated_ = true;

    if(param_->show_target)
    {
      markers_->setPosition("gripper_target", param_->x[0] + dx[0], param_->x[1] + dx[1], param_->x[2] + dx[2]);
      markers_->publish();
    }
    else
    {
      markers_->remove();
    }
  }
  catch(ahl_utils::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

void RedArm::control(const ros::TimerEvent&)
{
  try
  {
    ahl_utils::ScopedLock lock(mutex_);

    if(gazebo_interface_->subscribed())
    {
      Eigen::VectorXd q = gazebo_interface_->getJointStates();
      robot_->update(q);
      joint_updated_ = true;
    }

    if(!model_updated_) return;

    Eigen::VectorXd tau = Eigen::VectorXd::Zero(robot_->getDOF());
    controller_->computeGeneralizedForce(tau);
    gazebo_interface_->applyJointEfforts(tau);
  }
  catch(ahl_utils::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}
