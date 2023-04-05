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

#include "s_graphs/local_graph_generator.hpp"

namespace s_graphs {

LocalGraphGenerator::LocalGraphGenerator() {}

LocalGraphGenerator::~LocalGraphGenerator() {}

Rooms LocalGraphGenerator::get_current_room(
    const Rooms& room,
    const std::vector<VerticalPlanes>& x_vert_planes,
    const std::vector<VerticalPlanes>& y_vert_planes,
    const KeyFrame::Ptr keyframe,
    const std::vector<KeyFrame::Ptr>& keyframes,
    const std::vector<Rooms> rooms_vec) {
  Rooms current_room;
  for (const auto& room : rooms_vec) {
    if (is_keyframe_inside_room(room, x_vert_planes, y_vert_planes, keyframes.back())) {
      current_room = room;
      std::cout << "robot is currently in room with pose "
                << room.node->estimate().translation() << std::endl;
      return current_room;
    }
  }
}

std::vector<KeyFrame::Ptr> LocalGraphGenerator::get_keyframes_inside_room(
    const Rooms& current_room,
    const std::vector<VerticalPlanes>& x_vert_planes,
    const std::vector<VerticalPlanes>& y_vert_planes,
    const std::vector<KeyFrame::Ptr>& keyframes) {
  std::vector<s_graphs::KeyFrame::Ptr> room_keyframes;
  if (current_room.node != nullptr) {
    room_keyframes =
        get_room_keyframes(current_room, x_vert_planes, y_vert_planes, keyframes);
    std::cout << "Room has keyframes with size: " << room_keyframes.size() << std::endl;
  }
  return room_keyframes;
}

std::vector<const s_graphs::VerticalPlanes*> get_room_planes(
    const Rooms& current_room,
    const std::vector<VerticalPlanes>& x_vert_planes,
    const std::vector<VerticalPlanes>& y_vert_planes) {
  auto planes = obtain_planes_from_room(current_room, x_vert_planes, y_vert_planes);
  return planes;
}

void LocalGraphGenerator::generate_local_graph(
    const std::vector<s_graphs::KeyFrame::Ptr>& room_keyframes,
    const Rooms& current_room,
    std::shared_ptr<GraphSLAM> local_graph) {
  // loop over the keyframes
  for (const auto& keyframe : room_keyframes) {
  }
}

}  // namespace s_graphs
