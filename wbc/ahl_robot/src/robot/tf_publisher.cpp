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

#include "ahl_utils/exception.hpp"
#include "ahl_robot/definition.hpp"
#include "ahl_robot/robot/tf_publisher.hpp"

using namespace ahl_robot;

TfPublisher::TfPublisher()
{
}

void TfPublisher::publish(const RobotPtr& robot, bool publish_com)
{
  ros::Time current = ros::Time::now();

  for(auto it = std::begin(robot->getManipulatorName()); it != std::end(robot->getManipulatorName()); ++it)
  {
    ManipulatorPtr mnp = robot->getManipulator(*it);
    this->publish(mnp, current, publish_com);
  }
}

void TfPublisher::publish(const ManipulatorPtr& mnp, const ros::Time& current, bool publish_com)
{
  for(uint32_t i = 0; i < mnp->getLinkNum(); ++i)
  {
    geometry_msgs::TransformStamped tf_stamped;

    tf_stamped.header.frame_id = mnp->getLink(i)->parent;
    tf_stamped.child_frame_id  = mnp->getLink(i)->name;
    tf_stamped.header.stamp    = current;

    Eigen::Matrix3d R = mnp->getTransform(i).block(0, 0, 3, 3);
    Eigen::Quaterniond q(R);

    tf_stamped.transform.translation.x = mnp->getTransform(i).coeff(0, 3);
    tf_stamped.transform.translation.y = mnp->getTransform(i).coeff(1, 3);
    tf_stamped.transform.translation.z = mnp->getTransform(i).coeff(2, 3);

    tf_stamped.transform.rotation.x = q.x();
    tf_stamped.transform.rotation.y = q.y();
    tf_stamped.transform.rotation.z = q.z();
    tf_stamped.transform.rotation.w = q.w();
    transformBroadcaster().sendTransform(tf_stamped);

    if(!publish_com)
      continue;

    geometry_msgs::TransformStamped com_stamped;
    com_stamped.header.frame_id = mnp->getLink(i)->name;
    com_stamped.child_frame_id  = mnp->getLink(i)->name + "_com";
    com_stamped.header.stamp    = current;

    q = Eigen::Matrix3d::Identity();

    com_stamped.transform.translation.x = mnp->getLink(i)->C.coeff(0);
    com_stamped.transform.translation.y = mnp->getLink(i)->C.coeff(1);
    com_stamped.transform.translation.z = mnp->getLink(i)->C.coeff(2);

    com_stamped.transform.rotation.x = q.x();
    com_stamped.transform.rotation.y = q.y();
    com_stamped.transform.rotation.z = q.z();
    com_stamped.transform.rotation.w = q.w();
    transformBroadcaster().sendTransform(com_stamped);
  }
}
tf2_ros::TransformBroadcaster& TfPublisher::transformBroadcaster()
{
  static tf2_ros::TransformBroadcaster br;
  return br;
}
