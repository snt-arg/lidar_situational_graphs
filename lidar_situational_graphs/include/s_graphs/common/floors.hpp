/*
Copyright (c) 2023, University of Luxembourg
All rights reserved.

Redistributions and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*/

#ifndef FLOOR_HPP
#define FLOOR_HPP

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Eigen>
#include <boost/filesystem.hpp>
#include <g2o/vertex_floor.hpp>

using PointT = pcl::PointXYZI;
using PointNormal = pcl::PointXYZRGBNormal;

namespace g2o {
class VertexFloor;
class HyperGraph;
class SparseOptimizer;
}  // namespace g2o

namespace s_graphs {

/**
 * @brief Struct that contains information about floors
 *
 * @var id
 * @var graph_id
 * @var plane_x1_id, plane_x2_id, plane_y1_id, plane_y2_id
 * @var node
 */
class Floors {
 public:
  Floors() {}
  ~Floors() {}

  bool save(const std::string& directory, int sequential_id) {
    std::string floors_sub_directory = directory + "/" + std::to_string(sequential_id);
    if (!boost::filesystem::is_directory(floors_sub_directory)) {
      boost::filesystem::create_directory(floors_sub_directory);
    }

    std::ofstream ofs(floors_sub_directory + "/floor_data");

    ofs << "id\n";
    ofs << id << "\n";

    ofs << "sequential_id\n";
    ofs << sequential_id << "\n";

    ofs << "sequential_id\n";
    ofs << sequential_id << "\n";

    ofs << "floor_node\n";
    ofs << node->estimate().matrix() << "\n";

    ofs << "color\n";
    ofs << color[0] << " " << color[1] << " " << color[2] << "\n";

    ofs << "stair_keyframe_ids\n";
    for (const auto& id : stair_keyframe_ids) ofs << id << "\n";

    pcl::io::savePCDFileBinary(floors_sub_directory + "/floor_cloud.pcd", *floor_cloud);
    pcl::io::savePCDFileBinary(floors_sub_directory + "/floor_wall_cloud.pcd",
                               *floor_wall_cloud);
  }

  bool load(const std::string& directory, g2o::SparseOptimizer* local_graph) {}

 public:
  int id;
  int sequential_id;
  g2o::VertexFloor* node;  // node instance
  std::vector<double> color;
  std::vector<int> stair_keyframe_ids;
  pcl::PointCloud<PointT>::Ptr floor_cloud;
  pcl::PointCloud<PointNormal>::Ptr floor_wall_cloud;
};

}  // namespace s_graphs
#endif  // FLOORS_HPP
