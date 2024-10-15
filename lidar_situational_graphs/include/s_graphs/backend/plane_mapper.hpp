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

#ifndef PLANE_MAPPER_HPP
#define PLANE_MAPPER_HPP

#include <g2o/types/slam3d_addons/vertex_plane.h>
#include <math.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/format.hpp>
#include <cmath>
#include <g2o/edge_se3_point_to_plane.hpp>
#include <s_graphs/backend/graph_slam.hpp>
#include <s_graphs/backend/room_mapper.hpp>
#include <s_graphs/common/graph_utils.hpp>
#include <s_graphs/common/keyframe.hpp>
#include <s_graphs/common/plane_utils.hpp>
#include <s_graphs/common/planes.hpp>
#include <s_graphs/common/point_types.hpp>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "situational_graphs_msgs/msg/plane_data.hpp"
#include "situational_graphs_msgs/msg/planes_data.hpp"
#include "situational_graphs_msgs/msg/room_data.hpp"
#include "situational_graphs_msgs/msg/rooms_data.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace s_graphs {

/**
 * @brief
 */
class PlaneMapper {
 public:
  /**
   * @brief Contructor of class PlaneMapper
   *
   * @param private_nh
   */
  PlaneMapper(const rclcpp::Node::SharedPtr node, std::mutex& graph_mutex);
  ~PlaneMapper();

 public:
  /**
   * @brief
   *
   * @param covisibility_graph
   * @param keyframe
   * @param extracted_cloud_vec
   * @param x_vert_planes
   * @param y_vert_planes
   * @param hort_planes
   */
  void map_extracted_planes(
      std::shared_ptr<GraphSLAM>& covisibility_graph,
      KeyFrame::Ptr keyframe,
      const std::vector<pcl::PointCloud<PointNormal>::Ptr>& extracted_cloud_vec,
      std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      std::unordered_map<int, VerticalPlanes>& y_vert_planes,
      std::unordered_map<int, HorizontalPlanes>& hort_planes);

  /**
   * @brief
   *
   * @param keyframe
   * @param det_plane_body_frame
   * @return
   */
  g2o::Plane3D convert_plane_to_map_frame(const KeyFrame::Ptr& keyframe,
                                          const g2o::Plane3D& det_plane_body_frame);

  /**
   * @brief
   *
   * @param x_vert_planes
   * @param y_vert_planes
   * @param hort_planes
   * @param keyframe
   */
  void convert_plane_points_to_map(
      std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      std::unordered_map<int, VerticalPlanes>& y_vert_planes,
      std::unordered_map<int, HorizontalPlanes>& hort_planes,
      const KeyFrame::Ptr keyframe);

  /**
   * @brief
   *
   * @param x_vert_planes
   * @param y_vert_planes
   * @param hort_planes
   */
  void convert_plane_points_to_map(
      std::unordered_map<int, VerticalPlanes>& x_vert_planes,
      std::unordered_map<int, VerticalPlanes>& y_vert_planes,
      std::unordered_map<int, HorizontalPlanes>& hort_planes,
      std::tuple<std::vector<int>, std::vector<int>, std::vector<int>>
          updated_planes_tuple);

  /**
   * @brief
   *
   * @param plane_type
   * @param keyframe
   * @param det_plane
   * @param cloud_seg_body
   * @param x_vert_planes
   * @param y_vert_planes
   * @param hort_planes
   * @return
   */
  int associate_plane(const int& plane_type,
                      const KeyFrame::Ptr& keyframe,
                      const g2o::Plane3D& det_plane,
                      const pcl::PointCloud<PointNormal>::Ptr& cloud_seg_body,
                      const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
                      const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
                      const std::unordered_map<int, HorizontalPlanes>& hort_planes);

  /**
   * @brief
   *
   * @param current_floor_level
   * @param covisibility_graph
   * @param new_x_planes
   * @param new_y_planes
   * @param x_vert_planes
   * @param y_vert_planes
   */
  void factor_new_planes(const int& current_floor_level,
                         std::shared_ptr<GraphSLAM>& covisibility_graph,
                         const std::vector<g2o::VertexPlane*> new_x_planes,
                         const std::vector<g2o::VertexPlane*> new_y_planes,
                         std::unordered_map<int, VerticalPlanes>& x_vert_planes,
                         std::unordered_map<int, VerticalPlanes>& y_vert_planes);

  /**
   * @brief
   *
   * @param covisibility_graph
   * @param plane
   */
  template <typename T>
  void factor_saved_planes(const std::shared_ptr<GraphSLAM>& covisibility_graph,
                           const T& plane) {
    Eigen::Matrix3d plane_information_mat =
        Eigen::Matrix3d::Identity() * plane_information;

    for (size_t j = 0; j < plane.keyframe_node_vec.size(); j++) {
      g2o::Plane3D det_plane_body_frame =
          Eigen::Vector4d(plane.cloud_seg_body_vec[j]->back().normal_x,
                          plane.cloud_seg_body_vec[j]->back().normal_y,
                          plane.cloud_seg_body_vec[j]->back().normal_z,
                          plane.cloud_seg_body_vec[j]->back().curvature);

      auto edge = covisibility_graph->add_se3_plane_edge(plane.keyframe_node_vec[j],
                                                         plane.plane_node,
                                                         det_plane_body_frame.coeffs(),
                                                         plane_information_mat);
      covisibility_graph->add_robust_kernel(edge, "Huber", 1.0);
    }
  }

  /**
   * @brief
   *
   * @tparam T
   * @param covisibility_graph
   * @param plane
   */
  template <typename T>
  void factor_saved_duplicate_planes(
      const std::shared_ptr<GraphSLAM>& covisibility_graph,
      const std::unordered_map<int, T>& plane_vec,
      const T& plane) {
    Eigen::Matrix<double, 3, 3> information_2planes;
    information_2planes.setIdentity();
    information_2planes(0, 0) = dupl_plane_matching_information;
    information_2planes(1, 1) = dupl_plane_matching_information;
    information_2planes(2, 2) = dupl_plane_matching_information;

    std::set<g2o::HyperGraph::Edge*> plane_edges = plane.plane_node->edges();
    g2o::VertexPlane* plane2_node =
        plane_vec.find(plane.duplicate_id)->second.plane_node;

    if (!MapperUtils::check_plane_ids(plane_edges, plane2_node)) {
      auto edge_planes = covisibility_graph->add_2planes_edge(
          plane.plane_node, plane2_node, information_2planes);
      covisibility_graph->add_robust_kernel(edge_planes, "Huber", 1.0);
    }
  }

 private:
  /**
   * @brief
   *
   * @param covisibility_graph
   * @param plane
   * @return VerticalPlanes
   */
  VerticalPlanes add_plane(std::shared_ptr<GraphSLAM>& covisibility_graph,
                           g2o::VertexPlane* plane);

  /**
   * @brief
   *
   * @param covisibility_graph
   * @param keyframe
   * @param det_plane_body_frame
   * @param x_vert_planes
   * @param y_vert_planes
   * @param hort_planes
   * @return
   */
  int add_planes_to_graph(std::shared_ptr<GraphSLAM>& covisibility_graph,
                          const KeyFrame::Ptr& keyframe,
                          const pcl::PointCloud<PointNormal>::Ptr cloud_seg_body,
                          const g2o::Plane3D& det_plane_body_frame,
                          std::unordered_map<int, VerticalPlanes>& x_vert_planes,
                          std::unordered_map<int, VerticalPlanes>& y_vert_planes,
                          std::unordered_map<int, HorizontalPlanes>& hort_planes);

  /**
   * @brief
   *
   * @param covisibility_graph
   * @param plane_type
   * @param keyframe
   * @param det_plane_map_frame
   * @param det_plane_body_frame
   * @param x_vert_planes
   * @param y_vert_planes
   * @param hort_planes
   * @return
   */
  int sort_planes(std::shared_ptr<GraphSLAM>& covisibility_graph,
                  const int& plane_type,
                  const KeyFrame::Ptr& keyframe,
                  const pcl::PointCloud<PointNormal>::Ptr cloud_seg_body,
                  const g2o::Plane3D& det_plane_map_frame,
                  const g2o::Plane3D& det_plane_body_frame,
                  std::unordered_map<int, VerticalPlanes>& x_vert_planes,
                  std::unordered_map<int, VerticalPlanes>& y_vert_planes,
                  std::unordered_map<int, HorizontalPlanes>& hort_planes);

  /**
   * @brief
   *
   * @param covisibility_graph
   * @param plane_type
   * @param keyframe
   * @param det_plane_map_frame
   * @param det_plane_body_frame
   * @param x_vert_planes
   * @param y_vert_planes
   * @param hort_planes
   * @return
   */
  int factor_planes(std::shared_ptr<GraphSLAM>& covisibility_graph,
                    const int& plane_type,
                    const KeyFrame::Ptr& keyframe,
                    const pcl::PointCloud<PointNormal>::Ptr cloud_seg_body,
                    const g2o::Plane3D& det_plane_map_frame,
                    const g2o::Plane3D& det_plane_body_frame,
                    std::unordered_map<int, VerticalPlanes>& x_vert_planes,
                    std::unordered_map<int, VerticalPlanes>& y_vert_planes,
                    std::unordered_map<int, HorizontalPlanes>& hort_planes);

  /**
   * @brief
   *
   * @tparam T
   * @param covisibility_graph
   * @param det_plane_body_frame
   * @param det_plane_map_frame
   * @param keyframe
   * @param cloud_seg_body
   * @param planes
   * @return T
   */
  template <typename T>
  T add_new_plane(std::shared_ptr<GraphSLAM>& covisibility_graph,
                  const g2o::Plane3D& det_plane_body_frame,
                  const g2o::Plane3D& det_plane_map_frame,
                  const KeyFrame::Ptr& keyframe,
                  const pcl::PointCloud<PointNormal>::Ptr& cloud_seg_body,
                  std::unordered_map<int, T>& planes);

  /**
   * @brief
   *
   * @tparam T
   * @param match_id
   * @param keyframe
   * @param cloud_seg_body
   * @param planes
   * @return g2o::VertexPlane*
   */
  template <typename T>
  g2o::VertexPlane* update_plane(
      const int match_id,
      const KeyFrame::Ptr& keyframe,
      const pcl::PointCloud<PointNormal>::Ptr& cloud_seg_body,
      std::unordered_map<int, T>& planes);

  /**
   * @brief Get the matched planes object
   *
   * @tparam T
   * @param det_plane
   * @param keyframe
   * @param cloud_seg_body
   * @param planes
   */
  template <typename T>
  int get_matched_planes(const g2o::Plane3D& det_plane,
                         const KeyFrame::Ptr& keyframe,
                         const pcl::PointCloud<PointNormal>::Ptr& cloud_seg_body,
                         const std::unordered_map<int, T>& planes);

  /**
   * @brief
   *
   * @param plane_type
   * @param plane_id
   * @param keyframe
   * @param det_plane_map_frame
   * @param found_infinite_room
   * @param found_room
   * @param plane_id_pair
   */
  void retrieve_plane_properties(const int& plane_type,
                                 const int& plane_id,
                                 const KeyFrame::Ptr& keyframe,
                                 const g2o::Plane3D& det_plane_map_frame,
                                 bool& found_infinite_room,
                                 bool& found_room,
                                 plane_data_list& plane_id_pair);

  /**
   * @brief
   *
   * @tparam T
   * @param plane
   */
  template <typename T>
  void fill_plane_points(T& plane);

  struct PointNormalComparator {
    bool operator()(const PointNormal& a, const PointNormal& b) const {
      return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z);
    }
  };

 private:
  bool use_point_to_plane;
  double plane_information, dupl_plane_matching_information;
  double plane_dist_threshold;
  double plane_points_dist;
  double infinite_room_min_plane_length;
  double room_min_plane_length, room_max_plane_length;
  int min_plane_points, min_plane_points_opti;
  bool use_infinite_room_constraint;
  bool use_room_constraint;

 private:
  rclcpp::Node::SharedPtr node_obj;
  std::mutex& shared_graph_mutex;
};

}  // namespace s_graphs

#endif  // PLANE_MAPPER_HPP
