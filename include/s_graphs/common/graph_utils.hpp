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
#include <g2o/edge_loop_closure.hpp>
#include <g2o/edge_plane.hpp>
#include <g2o/edge_room.hpp>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/vertex_floor.hpp>
#include <s_graphs/backend/graph_slam.hpp>
#include <s_graphs/common/floors.hpp>
#include <s_graphs/common/infinite_rooms.hpp>
#include <s_graphs/common/keyframe.hpp>
#include <s_graphs/common/optimization_data.hpp>
#include <s_graphs/common/planes.hpp>
#include <s_graphs/common/rooms.hpp>
#include <unordered_set>

namespace s_graphs {

class GraphUtils {
 public:
  GraphUtils() {}

  /**
   * @brief
   *
   * @param covisibility_graph
   * @param compressed_graph
   * @return * void
   */
  void copy_graph(const std::shared_ptr<GraphSLAM>& covisibility_graph,
                  std::unique_ptr<GraphSLAM>& compressed_graph);

  /**
   * @brief
   *
   * @param window_size
   * @param covisibility_graph
   * @param compressed_graph
   * @param keyframes
   * @return * void
   */
  void copy_windowed_graph(const int window_size,
                           const std::shared_ptr<GraphSLAM>& covisibility_graph,
                           const std::unique_ptr<GraphSLAM>& compressed_graph,
                           const std::map<int, KeyFrame::Ptr>& keyframes);

  /**
   * @brief
   *
   * @param covisibility_graph
   * @param compressed_graph
   * @param keyframe_window
   * @return * int min_keyframe_id
   */
  int copy_keyframes_to_graph(const std::shared_ptr<GraphSLAM>& covisibility_graph,
                              const std::unique_ptr<GraphSLAM>& compressed_graph,
                              const std::map<int, KeyFrame::Ptr>& keyframe_window);

  /**
   * @brief
   *
   * @param covisibility_graph
   * @param compressed_graph
   * @param complete_keyframe_window
   * @return * std::vector<g2o::VertexSE3*>
   */
  std::vector<g2o::VertexSE3*> connect_keyframes(
      const std::shared_ptr<GraphSLAM>& covisibility_graph,
      const std::unique_ptr<GraphSLAM>& compressed_graph,
      const std::map<int, KeyFrame::Ptr>& complete_keyframe_window,
      bool& anchor_node_exists);

  /**
   * @brief
   *
   * @param covisibility_graph
   * @param compressed_graph
   * @return * std::unordered_set<g2o::VertexSE3*>
   */
  std::unordered_set<g2o::VertexSE3*> connect_keyframes_planes(
      const std::shared_ptr<GraphSLAM>& covisibility_graph,
      GraphSLAM* compressed_graph);

  /**
   * @brief
   *
   * @param covisibility_graph
   * @param compressed_graph
   * @return * void
   */
  void connect_planes_rooms(const std::shared_ptr<GraphSLAM>& covisibility_graph,
                            GraphSLAM* compressed_graph);

  /**
   * @brief
   *
   * @param covisibility_graph
   * @param compressed_graph
   * @return * void
   */
  void connect_rooms_floors(const std::shared_ptr<GraphSLAM>& covisibility_graph,
                            GraphSLAM* compressed_graph);

  /**
   * @brief
   *
   * @param compressed_graph
   * @param keyframes
   * @param x_vert_planes
   * @param y_vert_planes
   * @param rooms_vec
   * @param x_infinite_rooms
   * @param y_infinite_rooms
   * @param floors_vec
   */
  void update_graph(const std::unique_ptr<GraphSLAM>& compressed_graph,
                    std::map<int, KeyFrame::Ptr> keyframes,
                    std::vector<VerticalPlanes>& x_vert_planes,
                    std::vector<VerticalPlanes>& y_vert_planes,
                    std::vector<Rooms>& rooms_vec,
                    std::vector<InfiniteRooms>& x_infinite_rooms,
                    std::vector<InfiniteRooms>& y_infinite_rooms,
                    std::vector<Floors>& floors_vec);
};

}  // namespace s_graphs

#endif
