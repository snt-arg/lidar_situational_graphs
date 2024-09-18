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

#ifndef WALLS_HPP
#define WALLS_HPP

#include <pcl/common/distances.h>

#include <Eigen/Eigen>
#include <s_graphs/common/plane_utils.hpp>

namespace g2o {
class VertexWall;
class HyperGraph;
class SparseOptimizer;
}  // namespace g2o

namespace s_graphs {

/**
 * @brief Class that contains information about floors
 *
 * @var id
 * @var plane_x1_id, plane_x2_id, plane_y1_id, plane_y2_id
 * @var node
 */
class Walls {
 public:
  Walls() {}
  ~Walls() {}

  Walls(const Walls& old_wall, const bool deep_copy) {
    *this = old_wall;
    if (deep_copy) {
      node = new g2o::VertexWall();
      node->setEstimate(old_wall.node->estimate());
    }
  }

  Walls& operator=(const Walls& old_wall) {
    id = old_wall.id;
    plane1_id = old_wall.plane1_id;
    plane2_id = old_wall.plane2_id;
    floor_level = old_wall.floor_level;
    node = old_wall.node;
    return *this;
  }

  void save(const std::string& directory,
            const std::unordered_map<int, VerticalPlanes> vert_planes,
            const int sequential_id) {
    std::string parent_directory = directory.substr(0, directory.find_last_of("/\\"));

    std::string wall_sub_directory;
    wall_sub_directory = directory + "/" + std::to_string(sequential_id);

    if (!boost::filesystem::is_directory(wall_sub_directory)) {
      boost::filesystem::create_directory(wall_sub_directory);
    }

    std::ofstream ofs;
    ofs.open(wall_sub_directory + "/wall_data.txt");

    ofs << "id ";
    ofs << id << "\n";

    if (plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
      ofs << "plane_type ";
      ofs << "x"
          << "\n";
    } else {
      ofs << "plane_type ";
      ofs << "y"
          << "\n";
    }

    ofs << "plane1_id ";
    ofs << plane1_id << "\n";

    ofs << "plane2_id ";
    ofs << plane2_id << "\n";

    ofs << "floor_level ";
    ofs << floor_level << "\n";

    ofs << "wall_point ";
    ofs << wall_point.transpose() << "\n";

    ofs << "wall_pose ";
    ofs << node->estimate().transpose() << "\n";

    write_wall_data_to_csv(directory, vert_planes);
    // write_wall_points_data_to_csv(directory, vert_planes);
  }

  bool load(const std::string& directory,
            const std::shared_ptr<GraphSLAM>& covisibility_graph) {
    std::ifstream ifs;
    ifs.open(directory + "/wall_data.txt");

    Eigen::Vector3d wall_pose;
    while (!ifs.eof()) {
      std::string token;
      ifs >> token;

      if (token == "id") {
        ifs >> id;
      } else if (token == "plane_type") {
        std::string type;
        ifs >> type;
        if (type == "x")
          plane_type = PlaneUtils::plane_class::X_VERT_PLANE;
        else
          plane_type = PlaneUtils::plane_class::Y_VERT_PLANE;
      } else if (token == "plane1_id") {
        ifs >> plane1_id;
      } else if (token == "plane2_id") {
        ifs >> plane2_id;
      } else if (token == "floor_level") {
        ifs >> floor_level;
      } else if (token == "wall_point") {
        for (int i = 0; i < 3; i++) {
          ifs >> wall_point[i];
        }
      } else if (token == "wall_pose") {
        for (int i = 0; i < 3; i++) {
          ifs >> wall_pose[i];
        }
      }
    }

    g2o::VertexWall* wall_node(new g2o::VertexWall());
    wall_node->setId(id);
    wall_node->setEstimate(wall_pose);
    node = covisibility_graph->copy_wall_node(wall_node);

    return true;
  }

  void write_wall_data_to_csv(
      const std::string wall_directory,
      const std::unordered_map<int, VerticalPlanes> vert_planes) {
    std::string file_path = wall_directory + "/wall_data.csv";
    bool file_exists = boost::filesystem::exists(file_path);
    std::ofstream csv_ofs(file_path, std::ios::out | std::ios::app);
    if (!file_exists) {
      csv_ofs << "id,level,"
                 "start_point_x,start_point_y,start_point_z,end_point_x,end_point_y,"
                 "end_point_z,length,height,thickness\n";
    }

    // get the planes belonging to wall
    auto plane1 = vert_planes.find(plane1_id);
    auto plane2 = vert_planes.find(plane2_id);

    // get the smallest plane and its p_min and p_max
    PointNormal plane1_p_min, plane1_p_max;
    double length_plane1 =
        pcl::getMaxSegment(*plane1->second.cloud_seg_map, plane1_p_min, plane1_p_max);
    double height_plane1 = std::abs(plane1_p_max.z - plane1_p_min.z);

    PointNormal plane2_p_min, plane2_p_max;
    double length_plane2 =
        pcl::getMaxSegment(*plane1->second.cloud_seg_map, plane2_p_min, plane2_p_max);
    double height_plane2 = std::abs(plane2_p_max.z - plane2_p_min.z);

    PointNormal p_min, p_max;
    double length, height;
    g2o::VertexPlane* plane_node;
    if (length_plane1 < length_plane2) {
      length = length_plane1, height = height_plane1;
      p_min = plane1_p_min, p_max = plane1_p_max;
      plane_node = plane1->second.plane_node;
    } else {
      length = length_plane1, height = height_plane1;
      p_min = plane2_p_min, p_max = plane2_p_max;
      plane_node = plane2->second.plane_node;
    }

    double wall_thickness = PlaneUtils::width_between_planes(
        plane1->second.plane_node->estimate().coeffs(),
        plane2->second.plane_node->estimate().coeffs());

    PointNormal p_min_new, p_max_new;
    p_min_new.x = p_min.x + ((wall_thickness / 2) * plane_node->estimate().coeffs()(0));
    p_min_new.y = p_min.y + ((wall_thickness / 2) * plane_node->estimate().coeffs()(1));

    p_max_new.x = p_max.x + ((wall_thickness / 2) * plane_node->estimate().coeffs()(0));
    p_max_new.y = p_max.y + ((wall_thickness / 2) * plane_node->estimate().coeffs()(1));

    // write the smallest plane data to csv
    csv_ofs << id << "," << floor_level << "," << p_min_new.x << "," << p_min_new.y
            << "," << p_min.z << "," << p_max_new.x << "," << p_max_new.y << ","
            << p_min.z << "," << length << "," << height << "," << wall_thickness
            << "\n";
    csv_ofs.close();
  }

  void write_wall_points_data_to_csv(
      const std::string wall_directory,
      const std::unordered_map<int, VerticalPlanes> vert_planes) {
    std::string file_path = wall_directory + "/wall_points.csv";
    bool file_exists = boost::filesystem::exists(file_path);
    std::ofstream csv_ofs(file_path, std::ios::out | std::ios::app);
    if (!file_exists) {
      csv_ofs << "id,x,y,z\n ";
    }

    // get the planes belonging to wall
    auto plane1 = vert_planes.find(plane1_id);
    auto plane2 = vert_planes.find(plane2_id);

    for (const auto& point : plane1->second.cloud_seg_map->points) {
      csv_ofs << id << "," << point.x << "," << point.y << "," << point.z << "\n";
    }

    for (const auto& point : plane2->second.cloud_seg_map->points) {
      csv_ofs << id << "," << point.x << "," << point.y << "," << point.z << "\n";
    }
    csv_ofs.close();
  }

 public:
  int id;
  PlaneUtils::plane_class plane_type;
  int plane1_id, plane2_id;
  int floor_level;
  Eigen::Vector3d wall_point;
  g2o::VertexWall* node;  // node instance
};

}  // namespace s_graphs
#endif  // WALLS_HPP
