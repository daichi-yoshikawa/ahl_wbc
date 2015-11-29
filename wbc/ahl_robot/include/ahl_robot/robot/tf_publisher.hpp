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

////////////////////////////////////////////////////
/// \file tf_publisher.hpp
/// \brief Declare ahl_robot::TfPublisher class
/// \author Daichi Yoshikawa
////////////////////////////////////////////////////

#ifndef __AHL_ROBOT_TF_PUBLISHER_HPP
#define __AHL_ROBOT_TF_PUBLISHER_HPP

#include <boost/shared_ptr.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "ahl_robot/robot/robot.hpp"

namespace ahl_robot
{

  /// Publish tf depending on the state of ahl_robot::Robot
  class TfPublisher
  {
  public:
    /// Constructor
    TfPublisher();

    /// Publish tf
    /// \param robot Shared pointer of robot of which you'd like to see the frames
    /// \param publish_com If true, it publishes frames attached to center of mass of each link
    void publish(const RobotPtr& robot, bool publish_com = true);
  private:
    /// Publish tf
    /// \param mnp Shared pointer of manipulator of which you'd like to see the frames
    /// \param current Current time
    /// \publish_com If true, it publishes frames attached to center of mass of each link
    void publish(const ManipulatorPtr& mnp, const ros::Time& current, bool publish_com);

    //! Singleton of transform broadcaster
    tf2_ros::TransformBroadcaster& transformBroadcaster();
  };

  typedef boost::shared_ptr<TfPublisher> TfPublisherPtr;
}

#endif /* __AHL_ROBOT_TF_PUBLISHER_HPP */
