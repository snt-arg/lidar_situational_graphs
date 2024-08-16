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

#ifndef PLANES_HPP
#define PLANES_HPP

#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/plane3d.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <pcl/common/distances.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Eigen>
#include <boost/filesystem.hpp>
#include <s_graphs/common/keyframe.hpp>

namespace s_graphs {
/**
 * @brief
 *
 * @param id
 * @param plane
 * @param cloud_seg_body_vec
 * @param cloud_seg_map
 * @param covariance
 * @param keyframe_node_vec
 * @param plane_node
 * @param color
 * @param floor_level
 */

using PointNormal = pcl::PointXYZRGBNormal;
class Planes {
 public:
  Planes() {}

  Planes(const Planes& old_plane, const bool deep_copy) {
    *this = old_plane;
    if (deep_copy) {
      plane_node = new g2o::VertexPlane();
      plane_node->setEstimate(old_plane.plane_node->estimate());
    }
  }
  virtual ~Planes() {}

  Planes& operator=(const Planes& old_plane) {
    id = old_plane.id;
    cloud_seg_body_vec = old_plane.cloud_seg_body_vec;
    cloud_seg_map = old_plane.cloud_seg_map;
    covariance = old_plane.covariance;
    keyframe_node_vec = old_plane.keyframe_node_vec;
    color = old_plane.color;
    floor_level = old_plane.floor_level;
    revit_id = old_plane.revit_id;
    plane_node = old_plane.plane_node;
    type = old_plane.type;
    wall_point = old_plane.wall_point;
    start_point = old_plane.start_point;
    length = old_plane.length;
    on_wall = old_plane.on_wall;
    return *this;
  }

  void save(const std::string& directory, char type, int sequential_id) {
    std::string parent_directory = directory.substr(0, directory.find_last_of("/\\"));
    write_plane_data_to_csv(directory);
    write_plane_points_to_csv(directory);

    std::string plane_sub_directory;
    plane_sub_directory = directory + "/" + std::to_string(sequential_id);

    if (!boost::filesystem::is_directory(plane_sub_directory)) {
      boost::filesystem::create_directory(plane_sub_directory);
    }

    std::ofstream ofs;
    if (type == 'x') {
      ofs.open(plane_sub_directory + "/x_plane_data.txt");
    } else if (type == 'y') {
      ofs.open(plane_sub_directory + "/y_plane_data.txt");
    } else if (type == 'hort') {
      ofs.open(plane_sub_directory + "/hort_plane_data.txt");
    }

    ofs << "id ";
    ofs << id << "\n";

    if (duplicate_id != -1) {
      ofs << "duplicate_id ";
      ofs << duplicate_id << "\n";
    }

    ofs << "Covariance ";
    ofs << covariance << "\n";

    ofs << "plane_node_id ";
    ofs << plane_node->id() << "\n";

    ofs << "plane_node_pose ";
    ofs << plane_node->estimate().coeffs().transpose() << "\n";

    ofs << "type ";
    ofs << type << "\n";

    ofs << "color ";
    ofs << color[0] << " " << color[1] << " " << color[2] << "\n";

    ofs << "floor_level ";
    ofs << floor_level << "\n";

    ofs << "fixed ";
    ofs << plane_node->fixed() << "\n";

    ofs << "keyframe_vec_node_ids ";
    for (size_t i = 0; i < keyframe_node_vec.size(); i++) {
      ofs << keyframe_node_vec[i]->id() << "\n";
    }
    for (size_t i = 0; i < cloud_seg_body_vec.size(); i++) {
      std::string filename =
          plane_sub_directory + "/cloud_seg_body_" + std::to_string(i) + ".pcd";
      pcl::io::savePCDFileBinary(filename, *cloud_seg_body_vec[i]);
    }
    pcl::io::savePCDFileBinary(plane_sub_directory + "/cloud_seg_map.pcd",
                               *cloud_seg_map);
    ofs.close();
  }

  bool load(const std::string& directory,
            const std::shared_ptr<GraphSLAM>& covisibility_graph,
            std::string type) {
    std::ifstream ifs;

    if (type == "x") {
      ifs.open(directory + "/x_plane_data.txt");
    } else if (type == "y") {
      ifs.open(directory + "/y_plane_data.txt");
    } else if (type == "hort") {
      ifs.open(directory + "/hort_plane_data.txt");
    }

    if (!ifs) {
      return false;
    }

    Eigen::Vector4d plane_coeffs;
    while (!ifs.eof()) {
      std::string token;
      ifs >> token;
      if (token == "id") {
        ifs >> id;
      } else if (token == "duplicate_id") {
        ifs >> duplicate_id;
      } else if (token == "Covariance") {
        Eigen::Matrix3d mat;
        for (int i = 0; i < 3; i++) {
          for (int j = 0; j < 3; j++) {
            ifs >> mat(i, j);
          }
        }
        covariance = mat;
      } else if (token == "keyframe_vec_node_ids") {
        std::vector<int> kf_ids;
        int kf_id;
        while (ifs >> kf_id) {
          kf_ids.push_back(kf_id);
        }
        for (size_t i = 0; i < kf_ids.size(); i++) {
          for (const auto& vertex_pair : covisibility_graph->graph->vertices()) {
            g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(vertex_pair.second);
            if (vertex && vertex->id() == kf_ids[i]) {
              // Found the vertex with the given keyframe_id
              keyframe_node_vec.push_back(vertex);
            }
          }
        }
      } else if (token == "plane_node_pose") {
        for (int i = 0; i < 4; i++) {
          ifs >> plane_coeffs[i];
        }
      } else if (token == "color") {
        Eigen::Vector3d color_values;
        for (int i = 0; i < 3; i++) {
          ifs >> color_values[i];
        }
        color = {color_values[0], color_values[1], color_values[2]};
      } else if (token == "floor_level") {
        ifs >> floor_level;
      } else if (token == "fixed") {
        int fixed;
        ifs >> fixed;
        if (fixed == 1) {
          plane_node->setFixed(true);
          std::cout << "plane set fixed true  !" << std::endl;
        }
      }
    }

    g2o::VertexPlane* node(new g2o::VertexPlane());
    node->setId(id);
    node->setEstimate(plane_coeffs);
    plane_node = covisibility_graph->copy_plane_node(node);

    for (int i = 0; i < keyframe_node_vec.size(); i++) {
      std::string filename = "/cloud_seg_body_" + std::to_string(i) + ".pcd";
      pcl::PointCloud<PointNormal>::Ptr body_cloud(new pcl::PointCloud<PointNormal>());
      pcl::io::loadPCDFile(directory + filename, *body_cloud);
      cloud_seg_body_vec.push_back(body_cloud);
    }

    pcl::PointCloud<PointNormal>::Ptr map_cloud(new pcl::PointCloud<PointNormal>());
    pcl::io::loadPCDFile(directory + "/cloud_seg_map.pcd", *map_cloud);
    cloud_seg_map = map_cloud;
    return true;
  }

  // write plane data to csv
  void write_plane_data_to_csv(const std::string plane_directory) {
    if (on_wall) {
      std::cout << "Not adding plane " << id << " as its on wall" << std::endl;
      return;
    }

    std::string file_path = plane_directory + "/plane_data.csv";
    bool file_exists = boost::filesystem::exists(file_path);
    std::ofstream csv_ofs(file_path, std::ios::out | std::ios::app);
    if (!file_exists) {
      csv_ofs << "id,level,"
                 "start_point_x,start_point_y,start_point_z,end_point_x,end_point_y,"
                 "end_point_z,length,height\n";
    }

    pcl::PointXYZRGBNormal p_min, p_max;
    double length = pcl::getMaxSegment(*cloud_seg_map, p_min, p_max);
    double height = std::abs(p_max.z - p_min.z);

    if (p_min.z > p_max.z) {
      p_min.z = p_max.z;
    }

    // assuming planes which dont belong to a wall have a thickness of 20cm
    double wall_thickness = 0.2;
    pcl::PointXYZRGBNormal p_min_new, p_max_new;
    p_min_new.x = p_min.x + ((wall_thickness / 2) * plane_node->estimate().coeffs()(0));
    p_min_new.y = p_min.y + ((wall_thickness / 2) * plane_node->estimate().coeffs()(1));

    p_max_new.x = p_max.x + ((wall_thickness / 2) * plane_node->estimate().coeffs()(0));
    p_max_new.y = p_max.y + ((wall_thickness / 2) * plane_node->estimate().coeffs()(1));

    csv_ofs << id << "," << floor_level << "," << p_min_new.x << "," << p_min_new.y
            << "," << p_min.z << "," << p_max_new.x << "," << p_max_new.y << ","
            << p_min.z << "," << length << "," << height << "\n";
    csv_ofs.close();
  }

  void write_plane_points_to_csv(const std::string plane_directory) {
    if (on_wall) {
      std::cout << "Not adding plane " << id << " points as its on wall" << std::endl;
      return;
    }

    std::string file_path = plane_directory + "/plane_points.csv";
    bool file_exists = boost::filesystem::exists(file_path);
    std::ofstream csv_ofs(file_path, std::ios::out | std::ios::app);
    if (!file_exists) {
      csv_ofs << "id,x,y,z\n";
    }

    for (const auto& point : cloud_seg_map->points) {
      csv_ofs << id << "," << point.x << "," << point.y << "," << point.z << "\n";
    }
    csv_ofs.close();
  }

 public:
  int id;
  int duplicate_id = -1;
  std::vector<pcl::PointCloud<PointNormal>::Ptr>
      cloud_seg_body_vec;  // vector of segmented points of the plane in local
                           // body frame
  std::vector<g2o::VertexSE3*> keyframe_node_vec;  // vector keyframe node instance
  pcl::PointCloud<PointNormal>::Ptr
      cloud_seg_map;           // segmented points of the plane in global map frame
  Eigen::Matrix3d covariance;  // covariance of the landmark
  std::vector<double> color;
  int floor_level;  // current floor level
  int revit_id;
  g2o::VertexPlane* plane_node = nullptr;  // node instance
  std::string type;                        // Type online or prior
  double length;                           // Length of plane
  bool matched = false;                    // Flag if matched with prior/online or not
  Eigen::Vector2d start_point =
      Eigen::Vector2d::Ones();  // start point of the PRIOR wall in revit
  Eigen::Vector3d wall_point;   // point used to calculate prior wall center
  bool on_wall = false;  // variable to check if a plane is already associated to a wall
};

class VerticalPlanes : public Planes {
 public:
  VerticalPlanes() : Planes() {}
  ~VerticalPlanes() {}

  // copy constructor
  VerticalPlanes(const VerticalPlanes& old_plane, const bool deep_copy = false)
      : Planes(old_plane, deep_copy) {}

  VerticalPlanes& operator=(const VerticalPlanes& old_plane) {
    if (this != &old_plane) {
      Planes::operator=(old_plane);
    }
    return *this;
  }
};

class HorizontalPlanes : public Planes {
 public:
  HorizontalPlanes() : Planes() {}
  ~HorizontalPlanes() {}

  // copy constructor
  HorizontalPlanes(const HorizontalPlanes& old_plane, const bool deep_copy = false)
      : Planes(old_plane, deep_copy) {}

  HorizontalPlanes& operator=(const HorizontalPlanes& old_plane) {
    if (this != &old_plane) {
      Planes::operator=(old_plane);
    }
    return *this;
  }
};

}  // namespace s_graphs

#endif  // PLANES_HPP
