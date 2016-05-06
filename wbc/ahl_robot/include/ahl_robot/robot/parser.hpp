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

#ifndef __AHL_ROBOT_PARSER_HPP
#define __AHL_ROBOT_PARSER_HPP

#include <fstream>
#include <string>
#include <memory>
#include <yaml-cpp/yaml.h>
#include "ahl_robot/robot/robot.hpp"

namespace ahl_robot
{

  namespace yaml_tag
  {
    static const std::string ROBOT_NAME     = "name";
    static const std::string ROBOT_XYZ      = "xyz";
    static const std::string ROBOT_RPY      = "rpy";
    static const std::string WORLD_FRAME    = "world_frame";
    static const std::string MANIPULATORS   = "manipulators";
    static const std::string MNP_NAME       = "name";
    static const std::string LINKS          = "links";
    static const std::string LINK_NAME      = "name";
    static const std::string PARENT         = "parent";
    static const std::string JOINT_TYPE     = "joint_type";
    static const std::string LINK_XYZ       = "xyz_in_parent";
    static const std::string LINK_RPY       = "rpy_in_parent";
    static const std::string MASS           = "mass";
    static const std::string INERTIA_MATRIX = "inertia_matrix_in_com";
    static const std::string CENTER_OF_MASS = "com_in_link";
    static const std::string VX_MAX         = "vx_max";
    static const std::string VY_MAX         = "vy_max";
    static const std::string VZ_MAX         = "vy_max";
    static const std::string VROLL_MAX      = "vroll_max";
    static const std::string VPITCH_MAX     = "vpitch_max";
    static const std::string VYAW_MAX       = "vyaw_max";
    static const std::string Q_MIN          = "q_min";
    static const std::string Q_MAX          = "q_max";
    static const std::string DQ_MAX         = "dq_max";
    static const std::string TAU_MAX        = "tau_max";
    static const std::string INIT_Q         = "init_q";
    static const std::string DIFFERENTIATOR                  = "differentiator";
    static const std::string DIFFERENTIATOR_UPDATE_RATE      = "update_rate";
    static const std::string DIFFERENTIATOR_CUTOFF_FREQUENCY = "cutoff_frequency";

    static const std::string MACRO_MANIPULATOR_DOF = "macro_manipulator_dof";
  } // namespace yaml_tag

  class Parser
  {
  public:
    explicit Parser() {}
    void load(const std::string& path, const RobotPtr& robot);

  private:
    void loadRobotInfo(const RobotPtr& robot);
    void loadManipulator(const RobotPtr& robot);
    void loadLinks(const YAML::Node& node, const ManipulatorPtr& mnp);

    void loadVector3d(const YAML::Node& node, const std::string& tag, Eigen::Vector3d& v);
    void loadMatrix3d(const YAML::Node& node, const std::string& tag, Eigen::Matrix3d& m);
    void setLinkToManipulator(const std::map<std::string, double>& init_q, const ManipulatorPtr& mnp);
    void computeTotalDOF(const RobotPtr& robot);

    void checkTag(const YAML::Node& node, const std::string& tag, const std::string& func);

    std::ifstream ifs_;
    YAML::Node node_;
  };

  using ParserPtr = std::shared_ptr<Parser>;

} // namespace ahl_robot

#endif // __AHL_ROBOT_PARSER_HPP
