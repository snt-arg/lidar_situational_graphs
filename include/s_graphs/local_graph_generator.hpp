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

#include <s_graphs/graph_slam.hpp>
#include <s_graphs/room_utils.hpp>
#include <s_graphs/rooms.hpp>

namespace s_graphs {

class LocalGraphGenerator {
 public:
  LocalGraphGenerator();
  ~LocalGraphGenerator();

 public:
  Rooms get_current_room(const Rooms& room,
                         const std::vector<VerticalPlanes>& x_vert_planes,
                         const std::vector<VerticalPlanes>& y_vert_planes,
                         const KeyFrame::Ptr keyframe,
                         const std::vector<KeyFrame::Ptr>& keyframes,
                         const std::vector<Rooms> rooms_vec);

  std::vector<KeyFrame::Ptr> get_keyframes_inside_room(
      const Rooms& current_room,
      const std::vector<VerticalPlanes>& x_vert_planes,
      const std::vector<VerticalPlanes>& y_vert_planes,
      const std::vector<KeyFrame::Ptr>& keyframes);

  std::vector<const s_graphs::VerticalPlanes*> get_room_planes(
      const Rooms& current_room,
      const std::vector<VerticalPlanes>& x_vert_planes,
      const std::vector<VerticalPlanes>& y_vert_planes);

  void generate_local_graph(const std::vector<s_graphs::KeyFrame::Ptr>& room_keyframes,
                            const Rooms& current_room,
                            std::shared_ptr<GraphSLAM> local_graph);
};
}  // namespace s_graphs

#endif  // LOCAL_GRAPH_GENERATOR_HPP
