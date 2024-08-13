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
 * @param plane_x1_id, plane_x2_id, plane_y1_id, plane_y2_id
 * @param node
 * @param floor_level
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

      // room_keyframes.reserve(old_room.room_keyframes.size());
      for (const auto &k : old_room.room_keyframes) {
        std::make_shared<KeyFrameSnapshot>(k.second);
      }
    }
  }

  Rooms &operator=(const Rooms &old_room) {
    id = old_room.id;
    prior_id = old_room.prior_id;
    plane_x1_id = old_room.plane_x1_id;
    plane_x2_id = old_room.plane_x2_id;
    plane_y1_id = old_room.plane_y1_id;
    plane_y2_id = old_room.plane_y2_id;
    floor_level = old_room.floor_level;
    sub_room = old_room.sub_room;
    cluster_array = old_room.cluster_array;

    node = old_room.node;
    room_keyframes = old_room.room_keyframes;
    local_graph = old_room.local_graph;
    matched = old_room.matched;

    return *this;
  }

  void save(const std::string &directory, int sequential_id) {
    std::string rooms_sub_directory = directory + "/" + std::to_string(sequential_id);
    if (!boost::filesystem::is_directory(rooms_sub_directory)) {
      boost::filesystem::create_directory(rooms_sub_directory);
    }

    std::ofstream ofs(rooms_sub_directory + "/room_data.txt");
    ofs << "id ";
    ofs << id << "\n";

    ofs << "plane_x1_id ";
    ofs << plane_x1_id << "\n";

    ofs << "plane_x2_id ";
    ofs << plane_x2_id << "\n";

    ofs << "plane_y1_id ";
    ofs << plane_y1_id << "\n";

    ofs << "plane_y2_id ";
    ofs << plane_y2_id << "\n";

    ofs << "room_node ";
    ofs << node->estimate().matrix() << "\n";

    if (!room_keyframes.empty()) {
      ofs << "room_keyframes_ids\n";
      for (const auto &room_keyframe : room_keyframes) {
        ofs << room_keyframe.first << "\n";
      }
    }

    ofs.close();
  }

  bool load(const std::string &directory,
            const std::shared_ptr<GraphSLAM> covisibility_graph) {
    std::ifstream ifs(directory + "/room_data.txt");
    if (!ifs) {
      return false;
    }

    Eigen::Isometry3d room_pose;
    while (!ifs.eof()) {
      std::string token;
      ifs >> token;
      if (token == "id") {
        ifs >> id;
      } else if (token == "plane_x1_id") {
        ifs >> plane_x1_id;
      } else if (token == "plane_x2_id") {
        ifs >> plane_x2_id;
      } else if (token == "plane_y1_id") {
        ifs >> plane_y1_id;
      } else if (token == "plane_y2_id") {
        ifs >> plane_y2_id;
      } else if (token == "room_node") {
        Eigen::Matrix4d room_pose = Eigen::Matrix4d::Identity();
        for (int i = 0; i < 4; i++) {
          for (int j = 0; j < 4; j++) {
            ifs >> room_pose(i, j);
          }
        }
      }
    }

    node = covisibility_graph->add_room_node(room_pose);
    node->setId(id);

    return true;
  }

 public:
  int id;
  int prior_id;
  int plane_x1_id, plane_x2_id;
  int plane_y1_id, plane_y2_id;
  int floor_level;
  bool sub_room;
  bool matched = false;
  visualization_msgs::msg::MarkerArray cluster_array;

  g2o::VertexRoom *node = nullptr;  // node instance in covisibility graph
  std::map<int, KeyFrame::Ptr> room_keyframes;
  std::shared_ptr<GraphSLAM> local_graph;
};

}  // namespace s_graphs
#endif  // ROOMS_HPP
