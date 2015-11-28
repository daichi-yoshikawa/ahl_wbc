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

#include <fstream>
#include <ros/ros.h>
#include "ahl_utils/exception.hpp"
#include "ahl_robot/robot/parser.hpp"
#include "ahl_robot/tf/tf_publisher.hpp"
#include "ahl_digital_filter/pseudo_differentiator.hpp"

using namespace ahl_robot;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "parser_test");
  ros::NodeHandle nh;

  try
  {
    std::string name = "youbot";
    RobotPtr robot = RobotPtr(new Robot(name));

    ParserPtr parser = ParserPtr(new Parser());
    std::string path = "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/ahl_robot/ahl_robot/yaml/youbot.yaml";
    parser->load(path, robot);

    ros::MultiThreadedSpinner spinner;

    TfPublisherPtr tf_publisher = TfPublisherPtr(new TfPublisher());

    const std::string mnp_name = "mnp";
    unsigned long cnt = 0;
    const double period = 0.001;
    ros::Rate r(1 / period);

    ahl_filter::DifferentiatorPtr differentiator = ahl_filter::DifferentiatorPtr(new ahl_filter::PseudoDifferentiator(period, 1.0));

    Eigen::VectorXd q = Eigen::VectorXd::Constant(robot->getDOF(mnp_name), 0.0);
    Eigen::VectorXd dq = Eigen::VectorXd::Constant(robot->getDOF(mnp_name), 0.0);
    differentiator->init(q, dq);

    Eigen::VectorXd pre_q = q;

    //std::ofstream ofs1;
    //ofs1.open("result1.hpp");
    //std::ofstream ofs2;
    //ofs2.open("result2.hpp");
    //std::ofstream ofs3;
    //ofs3.open("result3.hpp");

    while(ros::ok())
    {
      q = Eigen::VectorXd::Constant(robot->getDOF(mnp_name), 1.0);
      double coeff = 1.0 * sin(2.0 * M_PI * 0.1 * cnt * period);
      ++cnt;

      q = coeff * q;

      //q = Eigen::VectorXd::Constant(robot->getDOF(mnp_name), M_PI / 2.0);

      q.coeffRef(0) = 0.0;
      q.coeffRef(1) = 0.0;
      q.coeffRef(2) = 0.0;

      robot->update(q);
      robot->computeJacobian(mnp_name);
      robot->computeMassMatrix(mnp_name);

      differentiator->apply(q);

      Eigen::VectorXd dq1 = robot->getJointVelocity(mnp_name);

      Eigen::VectorXd dq2;
      differentiator->copyDerivativeValueTo(dq2);

      std::cout << "p     : " << q.transpose() << std::endl;
      std::cout << "pre_p : " << pre_q.transpose() << std::endl;

      Eigen::VectorXd dq3 = (q - pre_q) / period;
      pre_q = q;

      //std::cout << "dq1 : " << dq1.block(3, 0, dq.rows() - 3, 1).transpose() << std::endl;
      //std::cout << "dq2 : " << dq2.block(3, 0, dq.rows() - 3, 1).transpose() << std::endl;
      //std::cout << "dq3 : " << dq3.block(3, 0, dq.rows() - 3, 1).transpose() << std::endl;
      //std::cout << dq << std::endl << std::endl;
      //std::cout << cos(2.0 * M_PI * 0.1 * cnt * 0.1) << std::endl;

      tf_publisher->publish(robot, false);

/*
      ofs1 << cnt * period << " ";
      ofs2 << cnt * period << " ";
      ofs3 << cnt * period << " ";
      for(unsigned int i = 0; i < dq.rows() - 3; ++i)
      {
        ofs1 << dq1[i + 3] << " ";
        ofs2 << dq2[i + 3] << " ";
        ofs3 << dq3[i + 3] << " ";
      }
      ofs1 << std::endl;
      ofs2 << std::endl;
      ofs3 << std::endl;
*/
      r.sleep();
    }
  }
  catch(ahl_utils::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }

  return 0;
}
