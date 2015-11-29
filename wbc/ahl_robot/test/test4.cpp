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
Eigen::MatrixXd J;
Eigen::MatrixXd T0, T1, T2;

void calc()
{
  //std::cout << "M : " << std::endl << M.inverse() << std::endl;
  //          << "J : " << std::endl << J << std::endl << std::endl;
  Eigen::MatrixXd M_inv = M.inverse();
  Eigen::MatrixXd Jv = J.block(0, 0, 3, J.cols());
  Eigen::MatrixXd Lambda_inv = Jv * M_inv * Jv.transpose();
  Eigen::MatrixXd Lambda = Lambda_inv.inverse();
  Eigen::MatrixXd J_dyn_inv = M_inv * Jv.transpose() * Lambda;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(7, 7);
  Eigen::MatrixXd N = I - J_dyn_inv * Jv;

  //std::cout << N.transpose() << std::endl << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "parser_test");
  ros::NodeHandle nh;

  try
  {
    std::string name = "red_arm";
    RobotPtr robot = RobotPtr(new Robot(name));

    ParserPtr parser = ParserPtr(new Parser());
    std::string path = "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/ahl_robot/ahl_robot/yaml/red_arm.yaml";
    parser->load(path, robot);

    ros::MultiThreadedSpinner spinner;

    TfPublisherPtr tf_publisher = TfPublisherPtr(new TfPublisher());

    const std::string mnp_name = "mnp";
    unsigned long cnt = 0;
    ros::Rate r(10.0);

    while(ros::ok())
    {
      Eigen::VectorXd q = Eigen::VectorXd::Zero(robot->getDOF(mnp_name));
      ManipulatorPtr mnp = robot->getManipulator(mnp_name);

      double goal = sin(2.0 * M_PI * 0.2 * cnt * 0.1);
      ++cnt;
      for(unsigned int i = 0; i < q.rows(); ++i)
      {
        q.coeffRef(6) = M_PI / 4.0 * goal;
      }
      std::cout << M_PI / 4.0 * goal << std::endl;
      //q = Eigen::VectorXd::Constant(q.rows(), 0.0 * M_PI / 4.0);

      robot->update(q);
      robot->computeJacobian(mnp_name);
      robot->computeMassMatrix(mnp_name);
      M = robot->getMassMatrix(mnp_name);
      J = robot->getJacobian(mnp_name, "gripper");

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
