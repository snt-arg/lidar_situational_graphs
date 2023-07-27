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
 * @param cloud_seg_body
 * @param cloud_seg_body_vec
 * @param cloud_seg_map
 * @param covariance
 * @param keyframe_node
 * @param keyframe_node_vec
 * @param plane_node
 * @param color
 */

using PointNormal = pcl::PointXYZRGBNormal;
class Planes {
 public:
  Planes() {}

  Planes(const Planes &old_plane, const bool deep_copy) {
    *this = old_plane;
    if (deep_copy) {
      keyframe_node = new g2o::VertexSE3();
      keyframe_node->setEstimate(old_plane.keyframe_node->estimate());
      plane_node = new g2o::VertexPlane();
      plane_node->setEstimate(old_plane.plane_node->estimate());
    }
  }
  virtual ~Planes() {}

  Planes &operator=(const Planes &old_plane) {
    id = old_plane.id;
    plane = old_plane.plane;
    cloud_seg_body = old_plane.cloud_seg_body;
    cloud_seg_body_vec = old_plane.cloud_seg_body_vec;
    cloud_seg_map = old_plane.cloud_seg_map;
    covariance = old_plane.covariance;
    keyframe_node_vec = old_plane.keyframe_node_vec;
    color = old_plane.color;
    revit_id = old_plane.revit_id;
    keyframe_node = old_plane.keyframe_node;
    plane_node = old_plane.plane_node;

    return *this;
  }

 public:
  int id;
  g2o::Plane3D plane;
  pcl::PointCloud<PointNormal>::Ptr
      cloud_seg_body;  // segmented points of the plane in local body frame
  std::vector<pcl::PointCloud<PointNormal>::Ptr>
      cloud_seg_body_vec;  // vector of segmented points of the plane in local
                           // body frame
  pcl::PointCloud<PointNormal>::Ptr
      cloud_seg_map;           // segmented points of the plane in global map frame
  Eigen::Matrix3d covariance;  // covariance of the landmark
  std::vector<g2o::VertexSE3 *> keyframe_node_vec;  // vector keyframe node instance
  std::vector<double> color;
  int revit_id;
  g2o::VertexSE3 *keyframe_node = nullptr;  // keyframe node instance
  g2o::VertexPlane *plane_node = nullptr;   // node instance
  std::string type;                         // Type online or prior
  double length;
  Eigen::Vector2d start_point;
};

class VerticalPlanes : public Planes {
 public:
  VerticalPlanes() : Planes() {}
  ~VerticalPlanes() {}

  // copy constructor
  VerticalPlanes(const VerticalPlanes &old_plane, const bool deep_copy = false)
      : Planes(old_plane, deep_copy) {}

  VerticalPlanes &operator=(const VerticalPlanes &old_plane) {
    if (this != &old_plane) {
      Planes::operator=(old_plane);
    }
    return *this;
  }
  void save(const std::string &directory, char type) {
    if (!boost::filesystem::is_directory(directory)) {
      boost::filesystem::create_directory(directory);
    }
    std::string x_planes_directory = directory + '/' + "x_planes";

    std::string y_planes_directory = directory + '/' + "y_planes";
    if (type == 'x') {
      if (!boost::filesystem::is_directory(x_planes_directory)) {
        boost::filesystem::create_directory(x_planes_directory);
      }
      std::ofstream ofs(x_planes_directory + "/x_plane_data");
      ofs << "id\n";
      ofs << id << "\n";

      ofs << "Plane \n";
      ofs << plane.coeffs() << "\n";

      ofs << "Covariance\n";
      ofs << covariance << "\n";

      ofs << "keyframe_node_id\n";
      ofs << keyframe_node->id() << "\n";

      ofs << "keyframe_node_pose\n";
      ofs << keyframe_node->estimate().matrix() << "\n";

      ofs << "plane_node_id\n";
      ofs << plane_node->id() << "\n";

      ofs << "plane_node_pose\n";
      ofs << plane_node->estimate().coeffs() << "\n";

      ofs << "type\n";
      ofs << type << "\n";

      ofs << "color\n";
      ofs << color[0] << "\n";
      ofs << color[1] << "\n";
      ofs << color[2] << "\n";

      ofs << "keyframe_vec_node_ids\n";
      for (int i = 0; i < keyframe_node_vec.size(); i++) {
        ofs << keyframe_node_vec[i]->id() << "\n";
        std::cout << "keyframe id at :  " << i << "   " << keyframe_node_vec[i]->id()
                  << std::endl;
      }
      pcl::io::savePCDFileBinary(x_planes_directory + "/cloud_seg_map.pcd",
                                 *cloud_seg_map);
      pcl::io::savePCDFileBinary(x_planes_directory + "/cloud_seg_body.pcd",
                                 *cloud_seg_body);
      for (int i = 0; i < cloud_seg_body_vec.size(); i++) {
        std::string filename =
            x_planes_directory + "/cloud_seg_body_" + std::to_string(i) + ".pcd";
        pcl::io::savePCDFileBinary(filename, *cloud_seg_body_vec[i]);
      }
    } else if (type == 'y') {
      if (!boost::filesystem::is_directory(y_planes_directory)) {
        boost::filesystem::create_directory(y_planes_directory);
      }
      std::ofstream ofs(y_planes_directory + "/y_plane_data");
      ofs << "id\n";
      ofs << id << "\n";

      ofs << "Plane\n";
      ofs << plane.coeffs() << "\n";

      ofs << "Covariance\n";
      ofs << covariance << "\n";

      ofs << "keyframe_node_id\n";
      ofs << keyframe_node->id() << "\n";

      ofs << "keyframe_node_pose\n";
      ofs << keyframe_node->estimate().matrix() << "\n";

      ofs << "plane_node_id\n";
      ofs << plane_node->id() << "\n";

      ofs << "plane_node_pose\n";
      ofs << plane_node->estimate().coeffs() << "\n";

      ofs << "type\n";
      ofs << type << "\n";
      ofs << "color\n";
      ofs << color[0] << "\n";
      ofs << color[1] << "\n";
      ofs << color[2] << "\n";

      ofs << "keyframe_vec_node_ids\n";
      for (int i = 0; i < keyframe_node_vec.size(); i++) {
        ofs << keyframe_node_vec[i]->id() << "\n";
        std::cout << "keyframe id at :  " << i << "   " << keyframe_node_vec[i]->id()
                  << std::endl;
      }
      pcl::io::savePCDFileBinary(y_planes_directory + "/cloud_seg_map.pcd",
                                 *cloud_seg_map);
      pcl::io::savePCDFileBinary(y_planes_directory + "/cloud_seg_body.pcd",
                                 *cloud_seg_body);
      for (int i = 0; i < cloud_seg_body_vec.size(); i++) {
        std::string filename =
            y_planes_directory + "/cloud_seg_body_" + std::to_string(i) + ".pcd";
        pcl::io::savePCDFileBinary(filename, *cloud_seg_body_vec[i]);
      }
    }

    std::cout << "written" << std::endl;
  }
  bool load(const std::string &directory,
            g2o::SparseOptimizer *local_graph,
            std::string type) {
    std::cout << "inside load func" << std::endl;
    std::ifstream ifs;
    if (type == "y") {
      ifs.open(directory + "/y_plane_data");
    } else if (type == "x") {
      ifs.open(directory + "/x_plane_data");
    } else {
      std::cout << "plane type not mentioned !" << std::endl;
      return false;
    }
    if (!ifs) {
      return false;
    }
    while (!ifs.eof()) {
      std::string token;
      ifs >> token;
      if (token == "id") {
        ifs >> id;
        plane_node->setId(id);
      } else if (token == "Plane") {
        Eigen::Vector4d plane_coeffs;
        for (int i = 0; i < 4; i++) {
          ifs >> plane_coeffs[i];
        }
        plane.fromVector(plane_coeffs);
      } else if (token == "Covariance") {
        Eigen::Matrix3d mat;
        for (int i = 0; i < 3; i++) {
          for (int j = 0; j < 3; j++) {
            ifs >> mat(i, j);
          }
        }
        covariance = mat;
      } else if (token == "keyframe_vec_node_ids") {
        std::vector<int> ids;
        int id;
        while (ifs >> id) {
          ids.push_back(id);
        }
        std::cout << "before keyframe vec size : " << ids.size() << std::endl;
        for (const auto &vertex_pair : local_graph->vertices()) {
          g2o::VertexSE3 *vertex = dynamic_cast<g2o::VertexSE3 *>(vertex_pair.second);
          for (int i = 0; i < ids.size(); i++) {
            if (vertex && vertex->id() == ids[i]) {
              // Found the vertex with the given keyframe_id
              keyframe_node_vec.push_back(vertex);
              std::cout << "keyframe id : " << vertex->id() << std::endl;
            }
          }
        }
        std::cout << "loaded keyframe vec size : " << keyframe_node_vec.size()
                  << std::endl;
      }

      else if (token == "keyframe_node_id") {
        int node_id;
        ifs >> node_id;

        for (const auto &vertex_pair : local_graph->vertices()) {
          g2o::VertexSE3 *vertex = dynamic_cast<g2o::VertexSE3 *>(vertex_pair.second);
          if (vertex && vertex->id() == node_id) {
            // Found the vertex with the given keyframe_id
            keyframe_node = vertex;

            break;
          }
        }
        if (keyframe_node) {
        } else {
          // The vertex with the given keyframe_id was not found in the graph
          std::cout << "Vertex with keyframe_id " << node_id
                    << " not found in the graph." << std::endl;
        }

      } else if (token == "plane_node_pose") {
        Eigen::Vector4d plane_coeffs;
        for (int i = 0; i < 4; i++) {
          ifs >> plane_coeffs[i];
        }
        plane_node->setEstimate(plane_coeffs);
      } else if (token == "color") {
        Eigen::Vector3d color_values;
        for (int i = 0; i < 3; i++) {
          ifs >> color_values[i];
        }
        color = {color_values[0], color_values[1], color_values[2]};
      }
    }

    int cloud_seg_body_vec_size = 0;
    boost::filesystem::path dir(directory);
    for (boost::filesystem::directory_iterator it(dir);
         it != boost::filesystem::directory_iterator();
         ++it) {
      if (boost::filesystem::is_regular_file(*it) && it->path().extension() == ".pcd") {
        cloud_seg_body_vec_size++;
      }
    }

    for (int i = 0; i < cloud_seg_body_vec_size; ++i) {
      std::string filename = "/cloud_seg_body_" + std::to_string(i) + ".pcd";
      pcl::PointCloud<PointNormal>::Ptr body_cloud(new pcl::PointCloud<PointNormal>());
      pcl::io::loadPCDFile(directory + filename, *body_cloud);
      cloud_seg_body_vec.push_back(body_cloud);
    }
    pcl::PointCloud<PointNormal>::Ptr map_cloud(new pcl::PointCloud<PointNormal>());
    pcl::io::loadPCDFile(directory + "/cloud_seg_map.pcd", *map_cloud);
    cloud_seg_map = map_cloud;
    pcl::PointCloud<PointNormal>::Ptr body_cloud(new pcl::PointCloud<PointNormal>());
    pcl::io::loadPCDFile(directory + "/cloud_seg_body.pcd", *body_cloud);
    cloud_seg_body = body_cloud;
    return true;
  }
};

class HorizontalPlanes : public Planes {
 public:
  HorizontalPlanes() : Planes() {}
  ~HorizontalPlanes() {}

  // copy constructor
  HorizontalPlanes(const HorizontalPlanes &old_plane, const bool deep_copy = false)
      : Planes(old_plane, deep_copy) {}

  HorizontalPlanes &operator=(const HorizontalPlanes &old_plane) {
    if (this != &old_plane) {
      Planes::operator=(old_plane);
    }
    return *this;
  }
};

}  // namespace s_graphs

#endif  // PLANES_HPP
