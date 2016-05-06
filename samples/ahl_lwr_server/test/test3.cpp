/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Daichi Yoshikawa
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Daichi Yoshikawa nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Daichi Yoshikawa
 *
 *********************************************************************/

#include <mutex>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ahl_gazebo_interface/gazebo_interface.hpp>
#include <ahl_gazebo_interface/exception.hpp>
#include <ahl_robot/ahl_robot.hpp>
#include <ahl_utils/exception.hpp>
#include <ahl_utils/scoped_lock.hpp>
#include <ahl_robot_controller/robot_controller.hpp>
#include <ahl_robot_controller/tasks.hpp>

using namespace ahl_robot;
using namespace ahl_ctrl;

std::mutex mutex;
RobotPtr robot;
RobotControllerPtr controller;
bool updated = false;
TfPublisherPtr tf_pub;
TaskPtr gravity_compensation;
TaskPtr damping;
TaskPtr joint_control;
TaskPtr joint_limit;
TaskPtr position_control;
TaskPtr orientation_control;
ahl_gazebo_if::GazeboInterfacePtr gazebo_interface;
bool initialized = false;
bool joint_updated = false;

void updateModel(const ros::TimerEvent&)
{
  try
  {
    ahl_utils::ScopedLock lock(mutex);
    if(joint_updated)
    {
      robot->computeJacobian("mnp");
      robot->computeMassMatrix("mnp");
      controller->updateModel();
      updated = true;
      tf_pub->publish(robot);
    }
  }
  catch(ahl_utils::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
  catch(ahl_gazebo_if::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

void control(const ros::TimerEvent&)
{
  try
  {
    ahl_utils::ScopedLock lock(mutex);

    if(gazebo_interface->subscribed())
    {
      Eigen::VectorXd q = gazebo_interface->getJointStates();
      robot->update(q);
      joint_updated = true;
    }

    if(updated)
    {
      static long cnt = 0;

      if(initialized == false)
      {
        Eigen::VectorXd qd = Eigen::VectorXd::Constant(robot->getDOF("mnp"), M_PI / 4.0);
        double sin_val = 1.0;//std::abs(sin(2.0 * M_PI * 0.1 * cnt * 0.001));
        qd = sin_val * qd;
        joint_control->setGoal(qd);

        static int32_t reached = 0;
        if(robot->reached("mnp", qd, 2.2))
        {
          ++reached;

          if(reached > 500)
          {
            std::cout << "switch to task space control." << std::endl;

            initialized = true;
            controller->clearTask();
            controller->addTask(joint_control, 0);
            controller->addTask(gravity_compensation, 10);
            controller->addTask(position_control, 10);
            controller->addTask(orientation_control, 5);
            //controller->addTask(joint_limit, 100);

            joint_control->setGoal(qd);

            Eigen::Vector3d xd;
            xd << 0.8, 0.35, 1.0;
            position_control->setGoal(xd);

            Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
            double rad = 0.0;
            R << cos(rad), 0, sin(rad),
              0, 1, 0,
              -sin(rad), 0, cos(rad);
            orientation_control->setGoal(R);
          }
        }
        else
        {
          reached = 0;
        }
      }
      else
      {
        Eigen::Vector3d xd;
        xd << 0.8, 0.35, 1.0;
        xd.coeffRef(0) += 0.2 * cos(2.0 * M_PI * 0.2 * cnt * 0.001);
        xd.coeffRef(1) += 0.2 * sin(2.0 * M_PI * 0.2 * cnt * 0.001);
        //xd.coeffRef(2) += 0.2 * cos(2.0 * M_PI * 0.2 * cnt * 0.001);

        Eigen::VectorXd qd = Eigen::VectorXd::Constant(robot->getDOF("mnp"), M_PI / 4.0);
        qd[3] = -qd[3];
        qd[7] = -qd[7];
        joint_control->setGoal(qd);

        position_control->setGoal(xd);
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        double rad = 0.0; //M_PI / 4.0 * sin(2.0 * M_PI * 0.2 * cnt * 0.001) + M_PI / 4;
        R << cos(rad), 0, sin(rad),
          0, 1, 0,
          -sin(rad), 0, cos(rad);
        orientation_control->setGoal(R);
      }

      ++cnt;

      Eigen::VectorXd tau(robot->getDOF("mnp"));
      controller->computeGeneralizedForce(tau);

      gazebo_interface->applyJointEfforts(tau);
    }

  }
  catch(ahl_utils::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
  catch(ahl_gazebo_if::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "red_arm_server");
  ros::NodeHandle nh;

  ros::Timer timer_update_model = nh.createTimer(ros::Duration(0.01), updateModel);
  ros::Timer timer_control = nh.createTimer(ros::Duration(0.001), control);

  robot = std::make_shared<Robot>("red_arm2");
  ParserPtr parser = std::make_shared<Parser>();

  std::string path = "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/ahl_robot/ahl_robot/yaml/red_arm2.yaml";
  parser->load(path, robot);

  controller = std::make_shared<RobotController>();
  controller->init(robot);

  using namespace ahl_gazebo_if;
  gazebo_interface = std::make_shared<GazeboInterface>();
  gazebo_interface->addJoint("red_arm2::joint1");
  gazebo_interface->addJoint("red_arm2::joint2");
  gazebo_interface->addJoint("red_arm2::joint3");
  gazebo_interface->addJoint("red_arm2::joint4");
  gazebo_interface->addJoint("red_arm2::joint5");
  gazebo_interface->addJoint("red_arm2::joint6");
  gazebo_interface->addJoint("red_arm2::joint7");
  gazebo_interface->addJoint("red_arm2::joint8");
  gazebo_interface->addJoint("red_arm2::joint9");
  gazebo_interface->addJoint("red_arm2::joint10");
  gazebo_interface->addJoint("red_arm2::joint11");
  gazebo_interface->connect();

  tf_pub = std::make_shared<TfPublisher>();

  ManipulatorPtr mnp = robot->getManipulator("mnp");

  gravity_compensation = std::make_shared<GravityCompensation>(robot);
  damping = std::make_shared<Damping>(robot);
  joint_control = std::make_shared<JointControl>(mnp);
  joint_limit = std::make_shared<JointLimit>(mnp, 0.087);
  position_control = std::make_shared<PositionControl>(mnp, "gripper", 0.001);
  orientation_control = std::make_shared<OrientationControl>(mnp, "gripper", 0.001);

  controller->addTask(gravity_compensation, 0);
  controller->addTask(joint_control, 0);

  ros::MultiThreadedSpinner spinner;
  spinner.spin();

  return 0;
}
