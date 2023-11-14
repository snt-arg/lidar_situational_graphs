#include "s_graphs/common/room_utils.hpp"

#include <string>

#include "pcl/filters/voxel_grid.h"

/**
 * Obtain the vertical planes that form the boundaries of a given room.
 * @param room The room whose planes are to be obtained.
 * @param x_vert_planes The list of vertical planes aligned with the x-axis.
 * @param y_vert_planes The list of vertical planes aligned with the y-axis.
 * @return A vector of pointers to the vertical planes that form the boundaries of the
 * room.
 */
std::vector<const s_graphs::VerticalPlanes*> obtain_planes_from_room(
    const s_graphs::Rooms& room,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& y_vert_planes) {
  std::vector<const s_graphs::VerticalPlanes*> planes;
  std::array<int, 4> ids = {room.plane_x1_id,
                            room.plane_x2_id,
                            room.plane_y1_id,
                            room.plane_y2_id};  // first index x_planes, second y_planes

  for (auto& id : ids) {
    // lambda funtion for finding a concrete id
    auto funct = [id](const s_graphs::VerticalPlanes& vectical_plane) {
      return id == vectical_plane.id;
    };
    // search over x_vert_planes
    if (planes.size() < 2) {
      auto it_x = x_vert_planes.find(id);
      if (it_x != x_vert_planes.end()) {
        planes.emplace_back(&(it_x->second));
      }
    } else {
      // search over y_vert_planes
      auto it_y = y_vert_planes.find(id);
      if (it_y != y_vert_planes.end()) {
        planes.emplace_back(&(it_y->second));
      }
    }
  }
  return planes;
}

bool is_SE3_inside_a_room(const Eigen::Isometry3d& pose,
                          const std::vector<PlaneGlobalRep>& planes) {
  const Eigen::Vector3d point = pose.translation();
  // check if the point is in front of all the planes
  for (auto& plane : planes) {
    const Eigen::Vector3d plane_point = plane.point;
    auto diff_point = point - plane_point;

    if (plane.normal.dot(diff_point) < 0) {
      return false;
    }
  }
  return true;
}

std::optional<Eigen::Vector3d> find_intersection(const Eigen::Vector3d& point1,
                                                 const Eigen::Vector3d& direction1,
                                                 const Eigen::Vector3d& point2,
                                                 const Eigen::Vector3d& direction2) {
  // Calculate the cross product of the two direction vectors
  Eigen::Vector3d cross_product = direction1.cross(direction2);

  Eigen::Matrix2f A;
  A << direction1(0), -direction1(1), direction2(0), -direction2(1);
  Eigen::Vector2f b;
  b << point2(0) - point1(0), point2(1) - point1(1);
  Eigen::Vector2f x = A.colPivHouseholderQr().solve(b);
  Eigen::Vector3d intersection_point;
  intersection_point << point1(0) + x(0) * direction1(0),
      point1(1) + x(0) * direction1(1), point1(2) + x(0) * direction1(2);

  //   // If the cross product is zero, the lines are parallel and do not intersect
  // if (cross_product.isZero()) {
  //   return std::nullopt;
  // }

  // // Calculate the parameter values for each line
  // double t1 = (cross_product.dot(direction2.cross(point1 - point2))) /
  //             cross_product.dot(direction1);
  // // double t2 = (direction1.cross(point1 - point2)).dot(cross_product) /
  // //             cross_product.dot(direction1);

  // // Calculate the intersection point
  // Eigen::Vector3d intersection_point = point1 + direction1 * t1;

  return intersection_point;
}

std::optional<Eigen::Isometry3d> obtain_global_centre_of_room(
    const std::vector<PlaneGlobalRep>& planes) {
  Eigen::Isometry3d room_centre;
  room_centre = room_centre.Identity();

  // We consider that the planes are related 2 by 2 (2 in x ,and the other 2 in y)
  // x procedure:
  auto p1 = planes[0].point, p2 = planes[1].point;
  auto p1_normal = planes[0].normal;
  auto vec_p2_p1 = p2 - p1;
  Eigen::Vector3d mid_point_x_vec_respect_p1 =
      p1 + p1_normal * (planes[0].normal.dot(vec_p2_p1) * 0.5);

  auto p3 = planes[2].point, p4 = planes[3].point;
  auto p3_normal = planes[2].normal;
  auto vec_p4_p3 = p4 - p3;
  Eigen::Vector3d mid_point_y_vec_respect_p3 =
      p3 + p3_normal * (planes[2].normal.dot(vec_p4_p3) * 0.5);

  auto rot = room_centre.rotation();
  rot.col(0) = p1_normal;
  rot.col(1) = p3_normal;
  rot.col(2) << p1_normal.cross(p3_normal);
  room_centre.linear() = rot;

  auto centre = find_intersection(
      mid_point_x_vec_respect_p1, p3_normal, mid_point_y_vec_respect_p3, p1_normal);
  if (!centre.has_value()) {
    return std::nullopt;
  }
  room_centre.translation() = centre.value();
  return room_centre;
}

std::vector<PlaneGlobalRep> obtain_global_planes_from_room(
    const s_graphs::Rooms& room,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& y_vert_planes) {
  std::vector<PlaneGlobalRep> plane_reps;
  auto planes = obtain_planes_from_room(room, x_vert_planes, y_vert_planes);
  if (planes.size() != 4) return {};
  // TODO:  DEBUG CAREFULLY
  for (auto& plane : planes) {
    PlaneGlobalRep plane_rep;
    plane_rep.normal = plane->plane.normal();
    Eigen::Vector3d glob_point = plane->plane.normal() * plane->plane.distance();
    plane_rep.point = glob_point;
    plane_reps.emplace_back(plane_rep);
  }
  return plane_reps;
}

std::set<std::pair<int, g2o::VertexSE3*>> obtain_keyframe_candidates_from_room(
    const s_graphs::Rooms& room,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& y_vert_planes) {
  std::set<std::pair<int, g2o::VertexSE3*>> keyframe_candidates;
  auto planes = obtain_planes_from_room(room, x_vert_planes, y_vert_planes);
  if (planes.size() != 4) return {};
  for (auto& plane : planes) {
    for (auto& keyframe : plane->keyframe_node_vec) {
      keyframe_candidates.insert({keyframe->id(), keyframe});
    }
  }
  return keyframe_candidates;
}

std::set<int> filter_keyframes_ids(
    const std::set<std::pair<int, g2o::VertexSE3*>>& candidate_keyframes,
    const std::vector<PlaneGlobalRep>& plane_reps) {
  std::set<int> keyframes_inside;

  for (auto& [id, keyframe] : candidate_keyframes) {
    if (!is_SE3_inside_a_room(keyframe->estimate(), plane_reps)) {
      continue;
    }
    keyframes_inside.insert({id});
  }

  return keyframes_inside;
}

std::set<g2o::VertexSE3*> publish_room_keyframes_ids(
    const s_graphs::Rooms& room,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& y_vert_planes) {
  std::set<g2o::VertexSE3*> keyframe_candidates;
  std::vector<PlaneGlobalRep> plane_reps;
  auto planes = obtain_planes_from_room(room, x_vert_planes, y_vert_planes);
  if (planes.size() != 4) return {};
  // TODO:  DEBUG CAREFULLY
  for (auto& plane : planes) {
    PlaneGlobalRep plane_rep;
    plane_rep.normal =
        plane->keyframe_node->estimate().linear() * plane->plane.normal();
    const auto glob_point = plane->keyframe_node->estimate() *
                            (-plane->plane.normal() * plane->plane.distance());
    plane_rep.point = glob_point;
    plane_reps.emplace_back(plane_rep);

    keyframe_candidates.insert(plane->keyframe_node_vec.begin(),
                               plane->keyframe_node_vec.end());
  }
  std::set<g2o::VertexSE3*> keyframes_inside;

  for (auto& keyframe_candidate : keyframe_candidates) {
    if (!is_SE3_inside_a_room(keyframe_candidate->estimate(), plane_reps)) {
      continue;
    }
    keyframes_inside.insert(keyframe_candidate);
  }

  return keyframes_inside;
}

std::set<g2o::VertexSE3*> filter_inside_room_keyframes(
    const s_graphs::Rooms& room,
    const std::set<g2o::VertexSE3*>& keyframes_candidates) {
  std::set<g2o::VertexSE3*> final_candidates;
  // room.node->estimate();

  return final_candidates;
}

template <typename KeyFramePtrVec>
KeyFramePtrVec obtain_keyframes_from_ids(const std::set<int>& id_list,
                                         const KeyFramePtrVec& _keyframes) {
  KeyFramePtrVec keyframes;
  for (auto& id : id_list) {
    auto it = _keyframes.find(id);
    if (it == _keyframes.end()) {
      continue;
    }
    keyframes.insert({id, it->second});
  }
  return keyframes;
}

static pcl::PointCloud<s_graphs::PointT>::Ptr filter_cloud(
    pcl::PointCloud<s_graphs::PointT>::Ptr cloud) {
  pcl::VoxelGrid<s_graphs::PointT> sor;
  pcl::PointCloud<s_graphs::PointT>::Ptr cloud_filtered(
      new pcl::PointCloud<s_graphs::PointT>);
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.1f, 0.1f, 0.1f);
  sor.filter(*cloud_filtered);

  cloud_filtered->width = cloud_filtered->size();
  cloud_filtered->height = 1;
  return cloud_filtered;
}

std::optional<
    std::pair<Eigen::Isometry3d, pcl::PointCloud<s_graphs::KeyFrame::PointT>::Ptr>>
generate_room_keyframe(
    const s_graphs::Rooms& room,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& y_vert_planes,
    const std::map<int, s_graphs::KeyFrame::Ptr>& keyframes) {
  auto global_planes =
      obtain_global_planes_from_room(room, x_vert_planes, y_vert_planes);
  auto room_centre = obtain_global_centre_of_room(global_planes);
  // print room centre and room centre estimate (4x4 matrix)
  // room.node->estimate();
  // std::cout << "Room centre: " << room_centre->matrix() << std::endl;
  // std::cout << "Room centre estimate: " << room.node->estimate().matrix() <<
  // std::endl;

  // If room centre coudn't be computed return
  if (!room_centre.has_value()) return {};

  room_centre = room.node->estimate();

  auto keyframe_candidates =
      obtain_keyframe_candidates_from_room(room, x_vert_planes, y_vert_planes);
  auto keyframes_ids = filter_keyframes_ids(keyframe_candidates, global_planes);
  auto keyframes_vec = obtain_keyframes_from_ids(keyframes_ids, keyframes);
  auto cloud = generate_room_pointcloud(
      room, room_centre.value(), keyframes_vec.begin(), keyframes_vec.end());

  std::vector<PlaneGlobalRep> local_plane_rep;
  for (auto& plane : global_planes) {
    PlaneGlobalRep local_plane;
    auto transform_point = room_centre->inverse() * plane.point;
    local_plane.point = transform_point;
    local_plane.normal = room_centre->linear().inverse() * plane.normal;
    local_plane_rep.emplace_back(local_plane);
  }

  double max_dist = 0;
  for (size_t i = 0; i < local_plane_rep.size() - 2; i++) {
    auto& plane_i_1 = local_plane_rep[i];
    auto& plane_i_2 = local_plane_rep[i + 2];
    auto intersection = find_intersection(
        plane_i_1.point, plane_i_2.normal, plane_i_2.point, plane_i_1.normal);
    if (intersection.has_value()) {
      auto intersec = intersection.value();
      intersec.z() = 0;
      auto dist = intersec.norm();
      if (dist > max_dist) {
        max_dist = dist;
      }
    }
  }
  auto room_pc = filter_room_pointcloud(cloud, max_dist);
  room_pc = filter_cloud(cloud);

  // // draw a sphere in the room centre to check if it is correct
  // // use spherical coordinates to generate the sphere
  // double radius = 0.1;
  // double theta = 0;
  // double phi = 0;
  // double step = 0.1;
  // for (; theta < 2 * M_PI; theta += step) {
  //   for (; phi < M_PI; phi += step) {
  //     s_graphs::PointT point;
  //     point.x = radius * sin(phi) * cos(theta);
  //     point.y = radius * sin(phi) * sin(theta);
  //     point.z = radius * cos(phi);
  //     room_pc->points.emplace_back(point);
  //   }
  // }
  // room_pc->width = room_pc->points.size();
  // room_pc->height = 1;

  // std::string initial_path =
  //     "/home/miguel/lux_stay_ws/ros2_ws/src/multirobot_sgraphs_server/pointcloud_data/";
  // pcl::io::savePCDFileASCII(initial_path + std::to_string(room.id) + "_room.pcd",
  //                           *room_pc);

  // // ext_room.cloud = transform_pointcloud<s_graphs::PointT>(ext_room.cloud,
  // // ext_room.centre.inverse());

  // return {{room_centre.value(), cloud}};
  return {{room_centre.value(), room_pc}};
  // room_centre.value(), cloud)};
}

std::map<int, s_graphs::KeyFrame::Ptr> get_room_keyframes(
    const s_graphs::Rooms& room,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& y_vert_planes,
    const std::map<int, s_graphs::KeyFrame::Ptr>& keyframes) {
  auto global_planes =
      obtain_global_planes_from_room(room, x_vert_planes, y_vert_planes);
  auto room_centre = obtain_global_centre_of_room(global_planes);

  auto keyframe_candidates =
      obtain_keyframe_candidates_from_room(room, x_vert_planes, y_vert_planes);

  auto keyframes_ids = filter_keyframes_ids(keyframe_candidates, global_planes);

  auto keyframes_vec = obtain_keyframes_from_ids(keyframes_ids, keyframes);

  return keyframes_vec;
}

bool is_keyframe_inside_room(
    const s_graphs::Rooms& room,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, s_graphs::VerticalPlanes>& y_vert_planes,
    const s_graphs::KeyFrame::Ptr keyframe) {
  auto global_planes =
      obtain_global_planes_from_room(room, x_vert_planes, y_vert_planes);

  if (is_SE3_inside_a_room(keyframe->estimate(), global_planes)) {
    return true;
  }

  return false;
}

graph_manager_msgs::msg::RoomKeyframe convertExtendedRoomToRosMsg(
    const ExtendedRooms& room) {
  graph_manager_msgs::msg::RoomKeyframe msg;
  msg.header.frame_id = "room";
  msg.id = room.id;
  pcl::toROSMsg(*room.cloud, msg.pointcloud);
  msg.pose = s_graphs::isometry2pose(room.centre);
  for (auto& keyframe : room.keyframes) {
    msg.keyframes_ids.emplace_back(keyframe->id());
  }
  msg.planes_ids.emplace_back(room.plane_x1_id);
  msg.planes_ids.emplace_back(room.plane_x2_id);
  msg.planes_ids.emplace_back(room.plane_y1_id);
  msg.planes_ids.emplace_back(room.plane_y2_id);

  return msg;
}

ExtendedRooms obtainExtendedRoomFromRosMsg(
    const graph_manager_msgs::msg::RoomKeyframe& msg) {
  ExtendedRooms room;
  room.id = msg.id;
  pcl::fromROSMsg(msg.pointcloud, *room.cloud);
  room.centre = s_graphs::pose2isometry(msg.pose);
  if (msg.planes_ids.size() != 4) {
    std::cout << "room doesnt contain 4 planes" << std::endl;
    return room;
  }
  room.plane_x1_id = msg.planes_ids[0];
  room.plane_x2_id = msg.planes_ids[1];
  room.plane_y1_id = msg.planes_ids[2];
  room.plane_y2_id = msg.planes_ids[3];

  // TODO: Fill these fields
  /* for (auto& keyframe : msg.keyframes_ids) {
    room.keyframes
  }
  msg.planes_ids.emplace_back(room.plane_x1_id);
  msg.planes_ids.emplace_back(room.plane_x2_id);
  msg.planes_ids.emplace_back(room.plane_y1_id);
  msg.planes_ids.emplace_back(room.plane_y2_id); */

  return room;
}

pcl::PointCloud<s_graphs::PointT>::Ptr filter_room_pointcloud(
    pcl::PointCloud<s_graphs::PointT>::Ptr cloud,
    double max_dist) {
  pcl::PointCloud<s_graphs::PointT>::Ptr filtered(
      new pcl::PointCloud<s_graphs::PointT>());

  // Easy filter, remove floor points -> Although this doesn't affect the
  // descriptor
  pcl::PassThrough<s_graphs::PointT> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  // pass.setFilterLimits(0.3, 3.0);
  pass.setFilterLimits(-5, 3);
  pass.filter(*filtered);
  if (max_dist) {
    pcl::PointCloud<s_graphs::PointT>::Ptr more_filtered(
        new pcl::PointCloud<s_graphs::PointT>());
    for (auto& pnt : filtered->points) {
      Eigen::Vector3d point = pnt.getVector3fMap().cast<double>();
      point.z() = 0;
      if (point.norm() > max_dist + 0.3) {
        continue;
      }
      more_filtered->points.emplace_back(pnt);
    }

    filtered = more_filtered;
  }
  filtered->width = filtered->size();
  filtered->height = 1;

  // pass.setInputCloud(filtered);
  // pass.setFilterFieldName("x");
  // pass.setFilterLimits(-5, 5);
  // pass.filter(*filtered);

  // pass.setInputCloud(filtered);
  // pass.setFilterFieldName("y");
  // pass.setFilterLimits(-5, 5);
  // pass.filter(*filtered);
  return filtered;
}

