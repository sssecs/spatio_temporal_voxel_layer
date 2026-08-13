/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2018, Simbe Robotics, Inc.
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
 *   * Neither the name of Simbe Robotics, Inc. nor the names of its
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
 * Author: Steve Macenski (steven.macenski@simberobotics.com)
 *                         stevenmacenski@gmail.com
 * Purpose: Structures for representing the robot's body as oriented boxes
 *          in the sensor frame so that points measured on the robot itself
 *          can be excluded from being marked as obstacles.
 *********************************************************************/

#ifndef SPATIO_TEMPORAL_VOXEL_LAYER__ROBOT_BODY_BOXES_HPP_
#define SPATIO_TEMPORAL_VOXEL_LAYER__ROBOT_BODY_BOXES_HPP_

// STL
#include <cmath>
#include <string>
#include <vector>
// Eigen
#include <Eigen/Dense>
// msgs
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace robot_body_boxes
{

// Oriented bounding box (OBB) describing a portion of the robot's body in the
// sensor frame.
struct BodyBox
{
  // Center of the box
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  // Orthonormal box axes stored as the rows of a rotation matrix
  Eigen::Matrix3d axes{Eigen::Matrix3d::Identity()};
  // Half extents along each box axis
  Eigen::Vector3d half_extents{Eigen::Vector3d::Zero()};

  // True if pt (in the same frame as the box) lies inside the box expanded
  // by margin
  bool Contains(const Eigen::Vector3d & pt, const double & margin) const
  {
    const Eigen::Vector3d local = axes * (pt - center);
    return std::abs(local.x()) <= half_extents.x() + margin &&
           std::abs(local.y()) <= half_extents.y() + margin &&
           std::abs(local.z()) <= half_extents.z() + margin;
  }
};

// Collection of boxes approximating the robot's body
struct RobotBodyBoxes
{
  std::vector<BodyBox> boxes;
  double margin = 0.0;

  // True if pt (in the same frame as the boxes) is inside any box
  bool IsPointInside(const Eigen::Vector3d & pt) const
  {
    for (const auto & box : boxes) {
      if (box.Contains(pt, margin)) {
        return true;
      }
    }
    return false;
  }
};

// Load the robot body boxes from a YAML file. Returns true on success.
bool LoadRobotBodyBoxes(const std::string & file_path, RobotBodyBoxes & boxes);

// Remove points inside any box from a point cloud (the boxes and the cloud
// must be in the same frame). The output cloud keeps the fields of the input.
void FilterCloud(
  const sensor_msgs::msg::PointCloud2 & in,
  sensor_msgs::msg::PointCloud2 & out,
  const RobotBodyBoxes & boxes);

}  // namespace robot_body_boxes

#endif  // SPATIO_TEMPORAL_VOXEL_LAYER__ROBOT_BODY_BOXES_HPP_
