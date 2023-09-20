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

#ifndef ROOMS_HPP
#define ROOMS_HPP

#include <g2o/types/slam3d_addons/plane3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Eigen>
#include <s_graphs/common/keyframe.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace g2o {
class VertexRoom;
class HyperGraph;
class SparseOptimizer;
}  // namespace g2o

namespace s_graphs {

/**
 * @brief
 *
 * @param id
 * @param plane_x1, plane_x2, plane_y1, plane_y2
 * @param plane_x1_id, plane_x2_id, plane_y1_id, plane_y2_id
 * @param neighbour_ids
 * @param node
 * @param sub_room
 * @param cluster_array
 */
class Rooms {
 public:
  Rooms() {}
  Rooms(const Rooms &old_room, const bool deep_copy = false) {
    *this = old_room;

    if (deep_copy) {
      node = new g2o::VertexRoom();
      node->setEstimate(old_room.node->estimate());

      plane_x1_node = new g2o::VertexPlane();
      plane_x1_node->setEstimate(old_room.plane_x1_node->estimate());
      plane_x2_node = new g2o::VertexPlane();
      plane_x2_node->setEstimate(old_room.plane_x2_node->estimate());
      plane_y1_node = new g2o::VertexPlane();
      plane_y1_node->setEstimate(old_room.plane_y1_node->estimate());
      plane_y2_node = new g2o::VertexPlane();
      plane_y2_node->setEstimate(old_room.plane_y2_node->estimate());

      room_keyframes.reserve(old_room.room_keyframes.size());
      for (const auto &k : old_room.room_keyframes) {
        std::make_shared<KeyFrameSnapshot>(k);
      }
    }
  }

  Rooms &operator=(const Rooms &old_room) {
    id = old_room.id;
    prior_id = old_room.prior_id;

    plane_x1 = old_room.plane_x1;
    plane_x2 = old_room.plane_x2;
    plane_y1 = old_room.plane_y1;
    plane_y2 = old_room.plane_y2;

    plane_x1_id = old_room.plane_x1_id;
    plane_x2_id = old_room.plane_x2_id;
    plane_y1_id = old_room.plane_y1_id;
    plane_y2_id = old_room.plane_y2_id;
    sub_room = old_room.sub_room;
    cluster_array = old_room.cluster_array;

    plane_x1_node = old_room.plane_x1_node;
    plane_x2_node = old_room.plane_x2_node;
    plane_y1_node = old_room.plane_y1_node;
    plane_y2_node = old_room.plane_y2_node;

    node = old_room.node;
    room_keyframes = old_room.room_keyframes;
    local_graph = old_room.local_graph;
    matched = old_room.matched;

    return *this;
  }
  bool save(const std::string &directory) {
    if (!boost::filesystem::is_directory(directory)) {
      boost::filesystem::create_directory(directory);
    }
    std::ofstream ofs(directory + "/room_data");
    ofs << "id\n";
    ofs << id << "\n";

    ofs << "Plane_x1 \n";
    ofs << plane_x1.coeffs() << "\n";

    ofs << "Plane_x2 \n";
    ofs << plane_x2.coeffs() << "\n";

    ofs << "Plane_y1 \n";
    ofs << plane_y1.coeffs() << "\n";

    ofs << "Plane_y2 \n";
    ofs << plane_y2.coeffs() << "\n";

    ofs << "plane_x1_id \n";
    ofs << plane_x1_id << "\n";

    ofs << "plane_x2_id \n";
    ofs << plane_x2_id << "\n";

    ofs << "plane_y1_id \n";
    ofs << plane_y1_id << "\n";

    ofs << "plane_y2_id \n";
    ofs << plane_y2_id << "\n";

    ofs << "plane_x1_node \n";
    ofs << plane_x1_node->estimate().coeffs() << "\n";

    ofs << "plane_x2_node \n";
    ofs << plane_x2_node->estimate().coeffs() << "\n";

    ofs << "plane_y1_node \n";
    ofs << plane_y1_node->estimate().coeffs() << "\n";

    ofs << "plane_y2_node \n";
    ofs << plane_y2_node->estimate().coeffs() << "\n";

    ofs << "room_node \n";
    ofs << node->estimate().matrix() << "\n";

    ofs << "room_keyframes_ids\n";
    std::cout << "room_keyframes.size() : " << room_keyframes.size() << std::endl;
    for (int i = 0; i < room_keyframes.size(); i++) {
      ofs << room_keyframes[i]->id() << "\n";
      std::cout << "keyframe id at :  " << i << "   " << room_keyframes[i]->id()
                << std::endl;
    }
    return true;
  }

 public:
  int id;
  int prior_id;
  g2o::Plane3D plane_x1;
  g2o::Plane3D plane_x2;
  g2o::Plane3D plane_y1;
  g2o::Plane3D plane_y2;
  int plane_x1_id;
  int plane_x2_id;
  int plane_y1_id;
  int plane_y2_id;
  bool sub_room;
  bool matched = false;
  visualization_msgs::msg::MarkerArray cluster_array;

  g2o::VertexPlane *plane_x1_node = nullptr;
  g2o::VertexPlane *plane_x2_node = nullptr;
  g2o::VertexPlane *plane_y1_node = nullptr;
  g2o::VertexPlane *plane_y2_node = nullptr;
  g2o::VertexRoom *node = nullptr;  // node instance in covisibility graph
  std::vector<KeyFrame::Ptr> room_keyframes;
  std::shared_ptr<GraphSLAM> local_graph;
};

}  // namespace s_graphs
#endif  // ROOMS_HPP
