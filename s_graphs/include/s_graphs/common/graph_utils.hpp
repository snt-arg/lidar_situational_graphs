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
  static void copy_graph(const std::shared_ptr<GraphSLAM>& covisibility_graph,
                         std::unique_ptr<GraphSLAM>& compressed_graph,
                         const std::map<int, KeyFrame::Ptr>& keyframes);

  /**
   * @brief
   *
   * @param current_floor_level
   * @param covisibility_graph
   * @param compressed_graph
   * @param keyframes
   * @param x_vert_planes
   * @param y_vert_planes
   * @param rooms_vec
   * @param x_infinite_rooms
   * @param y_infinite_rooms
   * @param floors_vec
   */
  static void copy_floor_graph(
      const int& current_floor_level,
      const std::shared_ptr<GraphSLAM>& covisibility_graph,
      std::unique_ptr<GraphSLAM>& compressed_graph,
      const std::map<int, KeyFrame::Ptr>& keyframes,
      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
      const std::unordered_map<int, Rooms>& rooms_vec,
      const std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
      const std::unordered_map<int, InfiniteRooms>& y_infinite_rooms,
      const std::map<int, Floors>& floors_vec);

  /**
   * @brief
   *
   * @param covisibility_graph
   * @param compressed_graph
   */
  static void copy_graph_vertices(const std::shared_ptr<GraphSLAM>& covisibility_graph,
                                  const std::unique_ptr<GraphSLAM>& compressed_graph);

  /**
   * @brief
   *
   * @param current_floor_level
   * @param covisibility_graph
   * @param compressed_graph
   * @param keyframes
   * @param x_vert_planes
   * @param y_vert_planes
   * @param rooms_vec
   * @param x_infinite_rooms
   * @param y_infinite_rooms
   * @param floors_vec
   * @param fix_kf
   * @return * void
   */
  static void copy_graph_vertices(
      const int& current_floor_level,
      const std::shared_ptr<GraphSLAM>& covisibility_graph,
      const std::unique_ptr<GraphSLAM>& compressed_graph,
      const std::map<int, KeyFrame::Ptr>& keyframes,
      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
      const std::unordered_map<int, Rooms>& rooms_vec,
      const std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
      const std::unordered_map<int, InfiniteRooms>& y_infinite_rooms,
      const std::map<int, Floors>& floors_vec,
      const bool& fix_kf = false);

  /**
   * @brief
   *
   * @param covisibility_graph
   * @param compressed_graph
   * @return * std::vector<g2o::VertexSE3*>
   */
  static std::vector<g2o::VertexSE3*> copy_graph_edges(
      const std::shared_ptr<GraphSLAM>& covisibility_graph,
      const std::unique_ptr<GraphSLAM>& compressed_graph);

  /**
   * @brief
   *
   * @param window_size
   * @param covisibility_graph
   * @param compressed_graph
   * @param keyframes
   * @return * void
   */
  static void copy_windowed_graph(const int window_size,
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
  static int copy_keyframes_to_graph(
      const std::shared_ptr<GraphSLAM>& covisibility_graph,
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
  static std::vector<g2o::VertexSE3*> connect_keyframes(
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
  static std::unordered_set<g2o::VertexSE3*> connect_keyframes_planes(
      const std::shared_ptr<GraphSLAM>& covisibility_graph,
      GraphSLAM* compressed_graph);

  /**
   * @brief
   *
   * @param filtered_k_vec
   * @param covisibility_graph
   * @param compressed_graph
   * @param keyframes
   * @return * void
   */
  static void connect_broken_keyframes(
      std::vector<g2o::VertexSE3*> filtered_k_vec,
      const std::shared_ptr<GraphSLAM>& covisibility_graph,
      const std::unique_ptr<GraphSLAM>& compressed_graph,
      const std::map<int, KeyFrame::Ptr>& keyframes);

  /**
   * @brief
   *
   * @param compressed_graph
   * @param fixed_keyframes_set
   */
  static void fix_and_connect_keyframes(
      GraphSLAM* compressed_graph,
      const std::unordered_set<g2o::VertexSE3*>& fixed_keyframes_set);

  /**
   * @brief
   *
   * @param covisibility_graph
   * @param compressed_graph
   * @return * void
   */
  static void connect_planes_rooms(const std::shared_ptr<GraphSLAM>& covisibility_graph,
                                   GraphSLAM* compressed_graph);

  /**
   * @brief
   *
   * @param covisibility_graph
   * @param compressed_graph
   * @return * void
   */
  static void connect_rooms_floors(const std::shared_ptr<GraphSLAM>& covisibility_graph,
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
  static void update_graph(const std::unique_ptr<GraphSLAM>& compressed_graph,
                           std::map<int, KeyFrame::Ptr> keyframes,
                           std::unordered_map<int, VerticalPlanes>& x_vert_planes,
                           std::unordered_map<int, VerticalPlanes>& y_vert_planes,
                           std::unordered_map<int, Rooms>& rooms_vec,
                           std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
                           std::unordered_map<int, InfiniteRooms>& y_infinite_rooms,
                           std::map<int, Floors>& floors_vec);

  /**
   * @brief Set the marginalize info object
   *
   * @param local_graph
   * @param covisibility_graph
   * @param room_keyframes
   * @return * void
   */
  static void set_marginalize_info(const std::shared_ptr<GraphSLAM>& local_graph,
                                   const std::shared_ptr<GraphSLAM>& covisibility_graph,
                                   const std::map<int, KeyFrame::Ptr>& room_keyframes);
  /**
   * @brief
   *
   * @param covisibility_graph
   * @param floor_level
   * @param keyframes
   * @param x_vert_planes
   * @param y_vert_planes
   * @param rooms_vec
   * @param x_infinite_rooms
   * @param y_infinite_rooms
   */
  static void update_node_floor_level(
      const int& first_keyframe_id,
      const int& current_floor_level,
      const std::map<int, KeyFrame::Ptr>& keyframes,
      std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      std::unordered_map<int, VerticalPlanes>& y_vert_planes,
      std::unordered_map<int, Rooms>& rooms_vec,
      std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
      std::unordered_map<int, InfiniteRooms>& y_infinite_rooms);

  /**
   * @brief
   *
   * @param vertex_se3
   * @return true
   * @return false
   */
  static bool get_keyframe_marg_data(g2o::VertexSE3* vertex_se3);

  /**
   * @brief Get the keyframe anchor data object
   *
   * @param vertex_se3
   * @return true
   * @return false
   */
  static bool get_keyframe_anchor_data(g2o::VertexSE3* vertex_se3);

  /**
   * @brief Get the keyframe stair data object
   *
   * @param vertex_se3
   * @return true
   * @return false
   */
  static bool get_keyframe_stair_data(g2o::VertexSE3* vertex_se3);

  /**
   * @brief
   * @param ids
   * @param keyframes
   */
  static void set_stair_keyframes(const std::vector<int>& ids,
                                  const std::map<int, KeyFrame::Ptr>& keyframes);
};

}  // namespace s_graphs

#endif
