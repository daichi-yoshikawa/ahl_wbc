#include "ahl_utils/exception.hpp"
#include "ahl_robot_samples/youbot/youbot.hpp"

using namespace ahl_sample;

YouBot::YouBot()
{
}

void YouBot::init()
{
  ros::NodeHandle local_nh("~");
  std::string yaml = "";

  local_nh.param<std::string>("robot_config", yaml, "/home/daichi/Work/catkin_ws/src/ahl_ros_pkgs/ahl_osf/samples/ahl_robot_samples/yaml/youbot.yaml");

  initRobot("youbot", yaml);

  controller_ = RobotControllerPtr(new RobotController());
  controller_->init(robot_);

  param_ = YouBotParamPtr(new YouBotParam());

  const double tread_width  = 0.3;
  const double wheel_base   = 0.471;
  const double wheel_radius = 0.05;
  mecanum_ = MecanumWheelPtr(
    new MecanumWheel(
      Eigen::Vector4d::Zero(), tread_width, wheel_base, wheel_radius));
  dqd_ = Eigen::Vector4d::Zero();

  ManipulatorPtr mnp = robot_->getManipulator("mnp");

  gravity_compensation_     = TaskPtr(new GravityCompensation(robot_));
  joint_control_            = TaskPtr(new JointControl(mnp));
  arm_position_control_     = TaskPtr(new PositionControl(mnp, "gripper", 0.001));
  arm_orientation_control_  = TaskPtr(new OrientationControl(mnp, "gripper", 0.001));
  base_position_control_    = TaskPtr(new PositionControl(mnp, "base_yaw", 0.001));
  base_orientation_control_ = TaskPtr(new OrientationControl(mnp, "base_yaw", 0.001));

  joint_control_->setGoal(param_->q);
  arm_position_control_->setGoal(param_->x_arm);
  arm_orientation_control_->setGoal(param_->R_arm);
  base_position_control_->setGoal(param_->x_base);
  base_orientation_control_->setGoal(param_->R_base);

  controller_->addTask(joint_control_, 0);
  //controller_->addTask(base_orientation_control_, 10);
  //controller_->addTask(base_position_control_, 11);
  controller_->addTask(arm_orientation_control_, 20);
  controller_->addTask(arm_position_control_, 21);
  controller_->addTask(gravity_compensation_, 21);

  gazebo_interface_ = GazeboInterfacePtr(new GazeboInterface());
  const double effort_time = 0.010;
  gazebo_interface_->addJoint("youbot::base_x_joint");
  gazebo_interface_->addJoint("youbot::base_y_joint");
  gazebo_interface_->addJoint("youbot::base_yaw_joint");
  gazebo_interface_->addJoint("youbot::joint1");
  gazebo_interface_->addJoint("youbot::joint2");
  gazebo_interface_->addJoint("youbot::joint3");
  gazebo_interface_->addJoint("youbot::joint4");
  gazebo_interface_->addJoint("youbot::joint5");
  gazebo_interface_->connect();

  gazebo_interface_wheel_ = GazeboInterfacePtr(new GazeboInterface());
  gazebo_interface_wheel_->addJoint("youbot::wheel_joint_fl");
  gazebo_interface_wheel_->addJoint("youbot::wheel_joint_fr");
  gazebo_interface_wheel_->addJoint("youbot::wheel_joint_bl");
  gazebo_interface_wheel_->addJoint("youbot::wheel_joint_br");
  gazebo_interface_wheel_->addLink("youbot", "wheel_link_fl");
  gazebo_interface_wheel_->addLink("youbot", "wheel_link_fr");
  gazebo_interface_wheel_->addLink("youbot", "wheel_link_bl");
  gazebo_interface_wheel_->addLink("youbot", "wheel_link_br");
  gazebo_interface_wheel_->connect();

  tf_pub_ = TfPublisherPtr(new TfPublisher());

  markers_ = MarkersPtr(new Markers());
  markers_->add(MarkerPtr(new Marker("gripper_target", "world")));
  markers_->add(MarkerPtr(new Marker("base_target", "world")));
  markers_->setColor(0, 1, 0, 0.5);
  markers_->setScale(0.1);
}

void YouBot::run()
{
  ros::NodeHandle nh;

  timer_update_model_ = nh.createTimer(ros::Duration(0.01), &YouBot::updateModel, this);
  timer_control_ = nh.createTimer(ros::Duration(0.001), &YouBot::control, this);
  timer_update_wheels_ = nh.createTimer(ros::Duration(0.01), &YouBot::updateWheels, this);

  ros::MultiThreadedSpinner spinner;
  spinner.spin();
}

void YouBot::updateModel(const ros::TimerEvent&)
{
  try
  {
    boost::mutex::scoped_lock lock(mutex_);
    if(!joint_updated_) return;

    joint_control_->setGoal(param_->q);
    arm_orientation_control_->setGoal(param_->R_arm);
    base_orientation_control_->setGoal(param_->R_base);

    const double amp = 0.07;
    const double f = 0.2;
    Eigen::Vector3d dx = Eigen::Vector3d::Zero();
    static double time = 0.0;

    if(param_->sin_x)
      dx[0] = amp * sin(2.0 * M_PI * f * time);
    if(param_->sin_y)
      dx[1] = amp * cos(2.0 * M_PI * f * time);
    if(param_->sin_z)
      dx[2] = amp * sin(2.0 * M_PI * f * time);

    time += 0.01;

    arm_position_control_->setGoal(param_->x_arm + dx);
    base_position_control_->setGoal(param_->x_base);

    robot_->computeJacobian();
    robot_->computeMassMatrix();
    controller_->updateModel();

    model_updated_ = true;

    if(param_->show_target)
    {
      markers_->setPosition("gripper_target", param_->x_arm[0] + dx[0], param_->x_arm[1] + dx[1], param_->x_arm[2] + dx[2]);
      markers_->setPosition("base_target", param_->x_base[0], param_->x_base[1], param_->x_base[2]);
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
  catch(ahl_ctrl::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

void YouBot::control(const ros::TimerEvent&)
{
  try
  {
    boost::mutex::scoped_lock lock(mutex_);

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

    Eigen::VectorXd qd, ddqd;
    controller_->simulate(0.001, tau, qd, dqd_, ddqd);
  }
  catch(ahl_utils::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
  catch(ahl_ctrl::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

void YouBot::updateWheels(const ros::TimerEvent& e)
{
  try
  {
    if(!gazebo_interface_wheel_->subscribed()) return;
    if(dqd_.rows() == 0) return;

    mecanum_->update(dqd_, 0.01);
    Eigen::Vector4d dq_wheel = mecanum_->getWheelVelocity() * 0.01;

    std::vector< Eigen::Quaternion<double> > quat_wheel;

    for(unsigned int i = 0; i < dq_wheel.rows(); ++i)
    {
      double rad = dq_wheel[i];

      Eigen::Quaternion<double> q;
      q.x() = 0.0;
      q.y() = sin(0.5 * rad);
      q.z() = 0.0;
      q.w() = cos(0.5 * rad);

      quat_wheel.push_back(q);
    }

    gazebo_interface_wheel_->rotateLink(quat_wheel);
  }
  catch(ahl_utils::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
  catch(ahl_ctrl::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}
