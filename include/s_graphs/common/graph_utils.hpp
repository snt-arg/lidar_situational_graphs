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

#ifndef __GRAPH_UTILS_HPP_
#define __GRAPH_UTILS_HPP_

#include <g2o/core/hyper_graph.h>
#include <g2o/core/optimizable_graph.h>
#include <g2o/types/slam3d/edge_se3.h>

#include <boost/bind.hpp>
#include <g2o/edge_plane.hpp>
#include <g2o/edge_room.hpp>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/vertex_floor.hpp>
#include <s_graphs/backend/graph_slam.hpp>
#include <s_graphs/common/floors.hpp>
#include <s_graphs/common/infinite_rooms.hpp>
#include <s_graphs/common/keyframe.hpp>
#include <s_graphs/common/planes.hpp>
#include <s_graphs/common/rooms.hpp>

namespace s_graphs {

class GraphUtils {
 public:
  GraphUtils() {}

  void copy_graph(const std::shared_ptr<GraphSLAM>& covisibility_graph,
                  std::unique_ptr<GraphSLAM>& global_graph);

  void update_graph(const std::unique_ptr<GraphSLAM>& global_graph,
                    std::vector<KeyFrame::Ptr> keyframes,
                    std::unordered_map<int, VerticalPlanes>& x_vert_planes,
                    std::unordered_map<int, VerticalPlanes>& y_vert_planes,
                    std::unordered_map<int, Rooms>& rooms_vec,
                    std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
                    std::unordered_map<int, InfiniteRooms>& y_infinite_rooms,
                    std::unordered_map<int, Floors>& floors_vec);
};

}  // namespace s_graphs

#endif
