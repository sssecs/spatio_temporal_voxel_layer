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
 * Purpose: Load robot body boxes from YAML and filter self-points out of
 *          sensor clouds
 *********************************************************************/

#include <exception>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "spatio_temporal_voxel_layer/robot_body_boxes.hpp"

namespace robot_body_boxes
{

/*****************************************************************************/
bool LoadRobotBodyBoxes(
  const std::string & file_path, RobotBodyBoxes & boxes)
/*****************************************************************************/
{
  try {
    const YAML::Node root = YAML::LoadFile(file_path);
    boxes.boxes.clear();
    boxes.margin = root["margin"] ? root["margin"].as<double>() : 0.0;

    const YAML::Node boxes_node = root["boxes"];
    if (!boxes_node || !boxes_node.IsSequence()) {
      return false;
    }

    for (const YAML::Node & box_node : boxes_node) {
      if (!box_node["center"] || !box_node["axes"] || !box_node["half_extents"]) {
        return false;
      }

      const auto center = box_node["center"].as<std::vector<double>>();
      const auto axes = box_node["axes"].as<std::vector<std::vector<double>>>();
      const auto half_extents = box_node["half_extents"].as<std::vector<double>>();
      if (center.size() != 3 || half_extents.size() != 3 || axes.size() != 3 ||
        axes[0].size() != 3 || axes[1].size() != 3 || axes[2].size() != 3)
      {
        return false;
      }

      BodyBox box;
      box.center = Eigen::Vector3d(center[0], center[1], center[2]);
      box.half_extents = Eigen::Vector3d(
        half_extents[0], half_extents[1], half_extents[2]);
      for (int i = 0; i != 3; ++i) {
        for (int j = 0; j != 3; ++j) {
          box.axes(i, j) = axes[i][j];
        }
      }
      boxes.boxes.push_back(box);
    }
    return !boxes.boxes.empty();
  } catch (const std::exception & ex) {
    return false;
  }
}

/*****************************************************************************/
void FilterCloud(
  const sensor_msgs::msg::PointCloud2 & in,
  sensor_msgs::msg::PointCloud2 & out,
  const RobotBodyBoxes & boxes)
/*****************************************************************************/
{
  // preserve the cloud structure, only dropping points inside the boxes
  out.header = in.header;
  out.height = 1;
  out.fields = in.fields;
  out.is_bigendian = in.is_bigendian;
  out.point_step = in.point_step;
  out.is_dense = in.is_dense;
  out.data.clear();
  out.data.reserve(in.data.size());

  if (in.point_step == 0) {
    out.width = 0;
    out.row_step = 0;
    return;
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(in, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(in, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(in, "z");

  size_t point_index = 0;
  for (; iter_x != iter_x.end();
    ++iter_x, ++iter_y, ++iter_z, ++point_index)
  {
    if (boxes.IsPointInside(Eigen::Vector3d(*iter_x, *iter_y, *iter_z))) {
      continue;
    }
    const size_t src = point_index * in.point_step;
    out.data.insert(
      out.data.end(),
      in.data.begin() + src, in.data.begin() + src + in.point_step);
  }

  out.width = out.data.size() / in.point_step;
  out.row_step = out.data.size();
}

}  // namespace robot_body_boxes
