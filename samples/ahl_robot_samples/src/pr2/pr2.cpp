#include "ahl_utils/exception.hpp"
#include "ahl_robot_samples/pr2/pr2.hpp"

using namespace ahl_sample;

PR2::PR2()
{

}

void PR2::init()
{
  ros::NodeHandle local_nh("~");

  std::string yaml;
  local_nh.param<std::string>("pr2/yaml", yaml, "");

  initRobot("pr2", yaml);

  controller_ = RobotControllerPtr(new RobotController());
  controller_->init(robot_);

  param_ = Pr2ParamPtr(new Pr2Param());

  ManipulatorPtr mnp_l = robot_->getManipulator("left_mnp");
  ManipulatorPtr mnp_r = robot_->getManipulator("right_mnp");

  gravity_compensation_     = TaskPtr(new GravityCompensation(robot_));
  joint_limit_l_            = TaskPtr(new JointLimit(mnp_l, 0.087));
  joint_limit_r_            = TaskPtr(new JointLimit(mnp_r, 0.087));
  joint_control_l_          = TaskPtr(new JointControl(mnp_l));
  joint_control_r_          = TaskPtr(new JointControl(mnp_r));
  position_control_l_       = TaskPtr(new PositionControl(mnp_l, "gripper_l_link", 0.001));
  position_control_r_       = TaskPtr(new PositionControl(mnp_r, "gripper_r_link", 0.001));
  orientation_control_l_    = TaskPtr(new OrientationControl(mnp_l, "gripper_l_link", 0.001));
  orientation_control_r_    = TaskPtr(new OrientationControl(mnp_r, "gripper_r_link", 0.001));

  joint_control_l_->setGoal(param_->q_l);
  joint_control_r_->setGoal(param_->q_r);
  orientation_control_l_->setGoal(param_->Rl);
  orientation_control_r_->setGoal(param_->Rr);
  position_control_l_->setGoal(param_->xl);
  position_control_r_->setGoal(param_->xr);

  controller_->addTask(joint_control_l_, 0);
  controller_->addTask(joint_control_r_, 0);
  controller_->addTask(orientation_control_l_, 20);
  controller_->addTask(orientation_control_r_, 21);
  controller_->addTask(position_control_l_, 30);
  controller_->addTask(position_control_r_, 31);
  controller_->addTask(gravity_compensation_, 40);
  //controller_->addTask(joint_limit_l_, 50);
  //controller_->addTask(joint_limit_r_, 51);

  gazebo_interface_ = GazeboInterfacePtr(new GazeboInterface());
  const double effort_time = 0.010;
  gazebo_interface_->addJoint("pr2::base_x_joint", effort_time);
  gazebo_interface_->addJoint("pr2::base_y_joint", effort_time);
  gazebo_interface_->addJoint("pr2::base_yaw_joint", effort_time);
  gazebo_interface_->addJoint("pr2::torso_joint", effort_time);
  gazebo_interface_->addJoint("pr2::shoulder_pan_l_joint", effort_time);
  gazebo_interface_->addJoint("pr2::shoulder_lift_l_joint", effort_time);
  gazebo_interface_->addJoint("pr2::upper_arm_l_joint", effort_time);
  gazebo_interface_->addJoint("pr2::elbow_l_joint", effort_time);
  gazebo_interface_->addJoint("pr2::forearm_l_joint", effort_time);
  gazebo_interface_->addJoint("pr2::wrist_flex_l_joint", effort_time);
  gazebo_interface_->addJoint("pr2::wrist_roll_l_joint", effort_time);
  gazebo_interface_->addJoint("pr2::shoulder_pan_r_joint", effort_time);
  gazebo_interface_->addJoint("pr2::shoulder_lift_r_joint", effort_time);
  gazebo_interface_->addJoint("pr2::upper_arm_r_joint", effort_time);
  gazebo_interface_->addJoint("pr2::elbow_r_joint", effort_time);
  gazebo_interface_->addJoint("pr2::forearm_r_joint", effort_time);
  gazebo_interface_->addJoint("pr2::wrist_flex_r_joint", effort_time);
  gazebo_interface_->addJoint("pr2::wrist_roll_r_joint", effort_time);

  gazebo_interface_->connect();

  tf_pub_ = TfPublisherPtr(new TfPublisher());

  markers_ = MarkersPtr(new Markers());
  markers_->add(MarkerPtr(new Marker("left_target", "world")));
  markers_->add(MarkerPtr(new Marker("right_target", "world")));
  markers_->setColor(0, 1, 0, 0.5);
  markers_->setScale(0.1);
}

void PR2::run()
{
  ros::NodeHandle nh;

  timer_update_model_ = nh.createTimer(ros::Duration(0.01), &PR2::updateModel, this);
  timer_control_ = nh.createTimer(ros::Duration(0.001), &PR2::control, this);

  ros::MultiThreadedSpinner spinner;
  spinner.spin();
}

void PR2::updateModel(const ros::TimerEvent&)
{
  try
  {
    boost::mutex::scoped_lock lock(mutex_);
    if(!joint_updated_) return;

    joint_control_l_->setGoal(param_->q_l);
    joint_control_r_->setGoal(param_->q_r);
    orientation_control_l_->setGoal(param_->Rl);
    orientation_control_r_->setGoal(param_->Rr);

    const double amp = 0.2;
    const double f = 0.3;
    Eigen::Vector3d dx_l = Eigen::Vector3d::Zero();
    Eigen::Vector3d dx_r = Eigen::Vector3d::Zero();
    static double time = 0.0;
    if(param_->sin_x_l)
      dx_l[0] = amp * sin(2.0 * M_PI * f * time);
    if(param_->sin_y_l)
      dx_l[1] = amp * cos(2.0 * M_PI * f * time);
    if(param_->sin_z_l)
      dx_l[2] = amp * sin(2.0 * M_PI * f * time);

    if(param_->sin_x_r)
      dx_r[0] = amp * cos(2.0 * M_PI * f * time);
    if(param_->sin_y_r)
      dx_r[1] = amp * sin(2.0 * M_PI * f * time);
    if(param_->sin_z_r)
      dx_r[2] = amp * cos(2.0 * M_PI * f * time);
    time += 0.01;

    position_control_l_->setGoal(param_->xl + dx_l);
    position_control_r_->setGoal(param_->xr + dx_r);

    robot_->computeJacobian();
    robot_->computeMassMatrix();
    controller_->updateModel();

    model_updated_ = true;

    if(param_->show_target)
    {
      markers_->setPosition("left_target", param_->xl[0] + dx_l[0], param_->xl[1] + dx_l[1], param_->xl[2] + dx_l[2]);
      markers_->setPosition("right_target", param_->xr[0] + dx_r[0], param_->xr[1] + dx_r[1], param_->xr[2] + dx_r[2]);
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

void PR2::control(const ros::TimerEvent&)
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

    Eigen::VectorXd qd, dqd, ddqd;
    controller_->simulate(0.001, tau, qd, dqd, ddqd);
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
