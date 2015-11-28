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

#ifndef __AHL_ROBOT_SAMPLES_ROBOT_DEMO_HPP
#define __AHL_ROBOT_SAMPLES_ROBOT_DEMO_HPP

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>

#include <ahl_gazebo_interface/gazebo_interface.hpp>
#include <ahl_gazebo_interface/exception.hpp>
#include <ahl_robot/ahl_robot.hpp>
#include <ahl_robot_controller/exception.hpp>
#include <ahl_robot_controller/robot_controller.hpp>
#include <ahl_robot_controller/tasks.hpp>

#include "ahl_robot_samples/marker/markers.hpp"

using namespace ahl_gazebo_if;
using namespace ahl_robot;
using namespace ahl_ctrl;

namespace ahl_sample
{

  class RobotDemo
  {
  public:
    RobotDemo() : joint_updated_(false), model_updated_(false) {}
    virtual ~RobotDemo() {}
    virtual void init() = 0;
    virtual void run()  = 0;

  protected:
    virtual void initRobot(const std::string& name, const std::string& yaml)
    {
      robot_ = RobotPtr(new Robot(name));
      ParserPtr parser = ParserPtr(new Parser());
      parser->load(yaml, robot_);
    }

    virtual void updateModel(const ros::TimerEvent&) = 0;
    virtual void control(const ros::TimerEvent&) = 0;

    boost::mutex mutex_;
    RobotPtr robot_;
    RobotControllerPtr controller_;
    bool joint_updated_;
    bool model_updated_;
    ros::Timer timer_update_model_;
    ros::Timer timer_control_;
    TfPublisherPtr tf_pub_;
    ahl_gazebo_if::GazeboInterfacePtr gazebo_interface_;
    MarkersPtr markers_;
  };

}

#endif /* __AHL_ROBOT_SAMPLES_ROBOT_DEMO_HPP */
