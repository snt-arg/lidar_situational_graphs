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

#include "s_graphs/backend/local_graph_generator.hpp"

namespace s_graphs {

LocalGraphGenerator::LocalGraphGenerator() {}

LocalGraphGenerator::~LocalGraphGenerator() {}

Rooms LocalGraphGenerator::get_current_room(
    const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    const KeyFrame::Ptr keyframe,
    const std::unordered_map<int, Rooms>& rooms_vec) {
  Rooms current_room;
  for (const auto& room : rooms_vec) {
    if (is_keyframe_inside_room(room.second, x_vert_planes, y_vert_planes, keyframe)) {
      current_room = room.second;
      std::cout << "robot is currently in room with pose "
                << room.second.node->estimate().translation() << std::endl;
    }
  }
  return current_room;
}

std::vector<KeyFrame::Ptr> LocalGraphGenerator::get_keyframes_inside_room(
    const Rooms& current_room,
    const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    const std::vector<KeyFrame::Ptr>& keyframes) {
  std::vector<s_graphs::KeyFrame::Ptr> room_keyframes;
  if (current_room.node != nullptr) {
    room_keyframes =
        get_room_keyframes(current_room, x_vert_planes, y_vert_planes, keyframes);
    std::cout << "Room has keyframes with size: " << room_keyframes.size() << std::endl;
  }

  std::vector<s_graphs::KeyFrame::Ptr> filtered_room_keyframes;
  for (const auto& room_keyframe : room_keyframes) {
    s_graphs::KeyFrame::Ptr new_room_keyframe =
        std::make_shared<KeyFrame>(*room_keyframe);
    filtered_room_keyframes.push_back(new_room_keyframe);
  }

  return filtered_room_keyframes;
}

std::vector<const s_graphs::VerticalPlanes*> get_room_planes(
    const Rooms& current_room,
    const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, VerticalPlanes>& y_vert_planes) {
  auto planes = obtain_planes_from_room(current_room, x_vert_planes, y_vert_planes);
  return planes;
}

void LocalGraphGenerator::generate_local_graph(
    std::unique_ptr<KeyframeMapper>& keyframe_mapper,
    std::shared_ptr<GraphSLAM> covisibility_graph,
    std::vector<KeyFrame::Ptr> filtered_keyframes,
    const Eigen::Isometry3d& odom2map,
    Rooms& current_room) {
  std::deque<KeyFrame::Ptr> new_room_keyframes;

  // check which keyframes already exist in the local graph and add only new ones
  for (const auto& filtered_keyframe : filtered_keyframes) {
    if (current_room.local_graph->graph->vertex(filtered_keyframe->id())) {
      continue;

    } else
      new_room_keyframes.push_back(filtered_keyframe);
  }

  keyframe_mapper->map_keyframes(current_room.local_graph,
                                 odom2map,
                                 new_room_keyframes,
                                 current_room.room_keyframes);
  // and to the local graph
  std::cout << "local graph keyframes size: "
            << current_room.local_graph->graph->vertices().size() << std::endl;

  // get the edges of the keyframe in the cov graph and add them to the local graph
}

}  // namespace s_graphs
