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

#include <ros/ros.h>
#include "ahl_utils/exception.hpp"
#include "ahl_robot/robot/parser.hpp"
#include "ahl_robot/robot/tf_publisher.hpp"

using namespace ahl_robot;

Eigen::MatrixXd M;
Eigen::MatrixXd J0, J1, J2;
Eigen::MatrixXd T0, T1, T2;

void calc()
{
  Eigen::Matrix2d m;
  double m1 = 3.0;
  double m2 = 1.5;
  double l1 = 0.5;
  double l2 = 0.4;
  double th1 = -M_PI / 2.0;
  double th2 =  M_PI / 2.0;
  double Izz1 = 0.05;
  double Izz2 = 0.025;
  m.coeffRef(0, 0) = m1 * l1 * l1 + Izz1 + m2 * (l1 * l1 + l2 * l2 + 2 * l1 * l2 * cos(th2)) + Izz2;
  m.coeffRef(0, 1) = m2 * (l2 * l2 + l1 * l2 * cos(th2)) + Izz2;
  m.coeffRef(1, 0) = m2 * (l2 * l2 + l1 * l2 * cos(th2)) + Izz2;
  m.coeffRef(1, 1) = l2 * l2 * m2 + Izz2;

  Eigen::Matrix2d j;
  j.coeffRef(0, 0) = -l1 * sin(th1) - l2 * sin(th1 + th2);
  j.coeffRef(0, 1) = -l2 * sin(th1 + th2);
  j.coeffRef(1, 0) = l1 * cos(th1) + l2 * cos(th1 + th2);
  j.coeffRef(1, 1) = l2 * cos(th1 + th2);

  std::cout << "M : " << std::endl << M << std::endl << std::endl
            << "m : " << std::endl << m << std::endl << std::endl
            << "Jv0 : " << std::endl << J0.block(0, 0, 2, 2) << std::endl
            << "Ref : " << std::endl << -l1 * sin(th1) << std::endl
            << l1 * cos(th1) << std::endl
            << "Jv1 : " << std::endl << J1.block(0, 0, 2, 2) << std::endl
            << "Ref : " << std::endl
            << -l1 * sin(th1) - l2* sin(th1 + th2) << " "
            << -l2 * sin(th1 + th2) << std::endl
            << l1 * cos(th1) + l2 * cos(th1 + th2) << " "
            << l2 * cos(th1 + th2) << std::endl << std::endl
            << "Jv2 : " << std::endl << J2.block(0, 0, 2, 2) << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "parser_test");
  ros::NodeHandle nh;

  try
  {
    std::string name = "rr_robot";
    RobotPtr robot = std::make_shared<Robot>(name);

    ParserPtr parser = std::make_shared<Parser>();
    std::string path = "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/ahl_robot/ahl_robot/yaml/rr_robot.yaml";
    parser->load(path, robot);

    ros::MultiThreadedSpinner spinner;

    TfPublisherPtr tf_publisher = std::make_shared<TfPublisher>();

    const std::string mnp_name = "mnp";
    unsigned long cnt = 0;
    ros::Rate r(10.0);

    while(ros::ok())
    {
      Eigen::VectorXd q = Eigen::VectorXd::Zero(robot->getDOF(mnp_name));

      q.coeffRef(0) = -M_PI / 2.0;
      q.coeffRef(1) =  M_PI / 2.0;

      robot->update(q);
      M = robot->getMassMatrix(mnp_name);
      J0 = robot->getJacobian(mnp_name, "link1");
      J1 = robot->getJacobian(mnp_name, "link2");
      J2 = robot->getJacobian(mnp_name, "gripper");
      ManipulatorPtr mnp = robot->getManipulator(mnp_name);

      calc();
      tf_publisher->publish(robot, false);
      r.sleep();
    }
  }
  catch(ahl_utils::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }

  return 0;
}
