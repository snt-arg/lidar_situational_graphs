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

// SPDX-License-Identifier: BSD-2-Clause

#ifndef LOCAL_GRAPH_GENERATOR_HPP
#define LOCAL_GRAPH_GENERATOR_HPP

#include <stdio.h>

#include <s_graphs/backend/graph_slam.hpp>
#include <s_graphs/backend/keyframe_mapper.hpp>
#include <s_graphs/common/room_utils.hpp>
#include <s_graphs/common/rooms.hpp>
#include <unordered_map>

namespace s_graphs {

class LocalGraphGenerator {
 public:
  LocalGraphGenerator();
  ~LocalGraphGenerator();

 public:
  Rooms get_current_room(const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
                         const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
                         const KeyFrame::Ptr keyframe,
                         const std::unordered_map<int, Rooms>& rooms_vec);

  std::vector<KeyFrame::Ptr> get_keyframes_inside_room(
      const Rooms& current_room,
      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
      const std::vector<KeyFrame::Ptr>& keyframes);

  std::vector<const s_graphs::VerticalPlanes*> get_room_planes(
      const Rooms& current_room,
      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      const std::unordered_map<int, VerticalPlanes>& y_vert_planes);

  void generate_local_graph(std::unique_ptr<KeyframeMapper>& keyframe_mapper,
                            std::shared_ptr<GraphSLAM> covisibility_graph,
                            std::vector<s_graphs::KeyFrame::Ptr> filtered_keyframes,
                            const Eigen::Isometry3d& odom2map,
                            Rooms& current_room);
};
}  // namespace s_graphs

#endif  // LOCAL_GRAPH_GENERATOR_HPP
