// SPDX-License-Identifier: BSD-2-Clause

#include <s_graphs/room_analyzer.hpp>

namespace s_graphs {

RoomAnalyzer::RoomAnalyzer(const ros::NodeHandle& private_nh, std::shared_ptr<PlaneUtils> plane_utils_ptr) {
  nh = private_nh;
  plane_utils = plane_utils_ptr;
  cloud_clusters_.clear();
  connected_subgraphs_.clear();
}

RoomAnalyzer::~RoomAnalyzer() {
  cloud_clusters_.clear();
  connected_subgraphs_.clear();
}

void RoomAnalyzer::analyze_skeleton_graph(const visualization_msgs::MarkerArray::Ptr& skeleton_graph_msg) {
  skeleton_graph_mutex.lock();
  cloud_clusters_.clear();
  connected_subgraphs_.clear();

  visualization_msgs::MarkerArray curr_connected_clusters_marker_array;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> curr_cloud_clusters;
  int subgraph_id = 0;
  std::vector<std::pair<int, int>> connected_subgraph_map;
  for(const auto& single_graph : skeleton_graph_msg->markers) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::string vertex_string = "connected_vertices_";
    size_t found = single_graph.ns.find(vertex_string);
    if(found != std::string::npos) {
      float r = rand() % 256;
      float g = rand() % 256;
      float b = rand() % 256;
      for(size_t i = 0; i < single_graph.points.size(); ++i) {
        pcl::PointXYZRGB pcl_point;
        pcl_point.x = single_graph.points[i].x;
        pcl_point.y = single_graph.points[i].y;
        pcl_point.z = 7.0;
        pcl_point.r = r;
        pcl_point.g = g;
        pcl_point.b = b;
        tmp_cloud_cluster->points.push_back(pcl_point);
      }
      // insert subgraph id in the seq
      tmp_cloud_cluster->header.seq = subgraph_id;
      curr_cloud_clusters.push_back(tmp_cloud_cluster);
      curr_connected_clusters_marker_array.markers.push_back(single_graph);
      subgraph_id++;
    }

    std::string edge_string = "connected_edges_";
    size_t edge_found = single_graph.ns.find(edge_string);
    if(edge_found != std::string::npos) {
      curr_connected_clusters_marker_array.markers.push_back(single_graph);
    }

    // get the connected subsgraphs
    std::string connected_subgraph_string = "subgraph_edges_";
    size_t found_connection = single_graph.ns.find(connected_subgraph_string);
    if(found_connection != std::string::npos) {
      // the position here encodes the two subgraph ids.
      int subgraph_1_id = single_graph.id >> 8;
      int subgraph_2_id = single_graph.id & (2 * 2 * 2 * 2 - 1);

      bool pair_exists = false;
      for(const auto& connected_graph_ids : connected_subgraph_map) {
        if(connected_graph_ids.first == subgraph_1_id && connected_graph_ids.second == subgraph_2_id) {
          pair_exists = true;
          continue;
        } else if(connected_graph_ids.first == subgraph_2_id && connected_graph_ids.second == subgraph_1_id) {
          pair_exists = true;
          continue;
        }
      }

      if(pair_exists) continue;

      std::pair<int, int> connected_subgraph;
      connected_subgraph = std::make_pair(subgraph_1_id, subgraph_2_id);
      connected_subgraph_map.push_back(connected_subgraph);
    }
  }

  cloud_clusters_ = curr_cloud_clusters;
  connected_subgraphs_ = connected_subgraph_map;
  connected_clusters_marker_array_ = curr_connected_clusters_marker_array;
  skeleton_graph_mutex.unlock();

  return;
}

geometry_msgs::Point RoomAnalyzer::get_room_length(const pcl::PointXY& p1, const pcl::PointXY& p2) {
  geometry_msgs::Point length;
  if(fabs(p1.x) > fabs(p2.x)) {
    length.x = fabs(p1.x - p2.x);
  } else {
    length.x = fabs(p2.x - p1.x);
  }

  if(fabs(p1.y) > fabs(p2.y)) {
    length.y = fabs(p1.y - p2.y);
  } else {
    length.y = fabs(p2.y - p1.y);
  }

  return length;
}

geometry_msgs::Point RoomAnalyzer::get_room_center(const s_graphs::PlaneData& x_plane1, const s_graphs::PlaneData& x_plane2, const s_graphs::PlaneData& y_plane1, const s_graphs::PlaneData& y_plane2) {
  geometry_msgs::Point center;
  Eigen::Vector3d vec_x, vec_y;
  Eigen::Vector3d x_plane1_eigen, x_plane2_eigen;
  Eigen::Vector3d y_plane1_eigen, y_plane2_eigen;

  x_plane1_eigen << x_plane1.nx, x_plane1.ny, x_plane1.nz;
  x_plane2_eigen << x_plane2.nx, x_plane2.ny, x_plane2.nz;

  y_plane1_eigen << y_plane1.nx, y_plane1.ny, y_plane1.nz;
  y_plane2_eigen << y_plane2.nx, y_plane2.ny, y_plane2.nz;

  if(fabs(x_plane1.d) > fabs(x_plane2.d)) {
    // double size = x_plane1.d - x_plane2.d;
    // center.x = (((size) / 2) + x_plane2.d);
    // center.x = center.x * fabs(x_plane2.nx);
    vec_x = (0.5 * (fabs(x_plane1.d) * x_plane1_eigen - fabs(x_plane2.d) * x_plane2_eigen)) + fabs(x_plane2.d) * x_plane2_eigen;
  } else {
    // double size = x_plane2.d - x_plane1.d;
    // center.x = (((size) / 2) + x_plane1.d);
    // center.x = center.x * fabs(x_plane1.nx);
    vec_x = (0.5 * (fabs(x_plane2.d) * x_plane2_eigen - fabs(x_plane1.d) * x_plane1_eigen)) + fabs(x_plane1.d) * x_plane1_eigen;
  }

  if(fabs(y_plane1.d) > fabs(y_plane2.d)) {
    // double size = y_plane1.d - y_plane2.d;
    // center.y = (((size) / 2) + y_plane2.d);
    // center.y = center.y * fabs(y_plane2.ny);
    vec_y = (0.5 * (fabs(y_plane1.d) * y_plane1_eigen - fabs(y_plane2.d) * y_plane2_eigen)) + fabs(y_plane2.d) * y_plane2_eigen;
  } else {
    // double size = y_plane2.d - y_plane1.d;
    // center.y = (((size) / 2) + y_plane1.d);
    // center.y = center.y * fabs(y_plane1.ny);
    vec_y = (0.5 * (fabs(y_plane2.d) * y_plane2_eigen - fabs(y_plane1.d) * y_plane1_eigen)) + fabs(y_plane1.d) * y_plane1_eigen;
  }

  Eigen::Vector3d final_vec = vec_x + vec_y;
  center.x = final_vec(0);
  center.y = final_vec(1);

  std::cout << "final room vec: " << final_vec << std::endl;
  return center;
}

geometry_msgs::Point RoomAnalyzer::get_corridor_center(int plane_type, pcl::PointXY p1, pcl::PointXY p2, s_graphs::PlaneData plane1, s_graphs::PlaneData plane2) {
  geometry_msgs::Point center;
  Eigen::Vector3d vec;
  Eigen::Vector3d plane1_eigen, plane2_eigen;
  plane1_eigen << plane1.nx, plane1.ny, plane1.nz;
  plane2_eigen << plane2.nx, plane2.ny, plane2.nz;

  if(plane_type == PlaneUtils::plane_class::X_VERT_PLANE) {
    if(fabs(plane1.d) > fabs(plane2.d)) {
      // double size = plane1.d - plane2.d;
      // center.x = (((size) / 2) + plane2.d);
      // center.x = center.x * plane2.nx;
      vec = (0.5 * (fabs(plane1.d) * plane1_eigen - fabs(plane2.d) * plane2_eigen)) + fabs(plane2.d) * plane2_eigen;
    } else {
      // double size = plane2.d - plane1.d;
      // center.x = (((size) / 2) + plane1.d);
      // center.x = center.x * plane1.nx;
      vec = (0.5 * (fabs(plane2.d) * plane2_eigen - fabs(plane1.d) * plane1_eigen)) + fabs(plane1.d) * plane1_eigen;
    }
    center.x = vec(0);

    if(fabs(p1.y) > fabs(p2.y)) {
      float size = p1.y - p2.y;
      center.y = (size / 2) + p2.y;
    } else {
      float size = p2.y - p1.y;
      center.y = (size / 2) + p1.y;
    }
    std::cout << "x corridor center:" << center.x << "; " << center.y << std::endl;
  }

  if(plane_type == PlaneUtils::plane_class::Y_VERT_PLANE) {
    if(fabs(plane1.d) > fabs(plane2.d)) {
      // double size = plane1.d - plane2.d;
      // center.y = (((size) / 2) + plane2.d);
      // center.y = center.y * plane2.ny;
      vec = (0.5 * (fabs(plane1.d) * plane1_eigen - fabs(plane2.d) * plane2_eigen)) + fabs(plane2.d) * plane2_eigen;
    } else {
      // double size = plane2.d - plane1.d;
      // center.y = (((size) / 2) + plane1.d);
      // center.y = center.y * plane1.ny;
      vec = (0.5 * (fabs(plane2.d) * plane2_eigen - fabs(plane1.d) * plane1_eigen)) + fabs(plane1.d) * plane1_eigen;
    }
    center.y = vec(1);

    if(fabs(p1.x) > fabs(p2.x)) {
      float size = p1.x - p2.x;
      center.x = (size / 2) + p2.x;
    } else {
      float size = p2.x - p1.x;
      center.x = (size / 2) + p1.x;
    }
    std::cout << "y corridor center:" << center.x << "; " << center.y << std::endl;
  }

  return center;
}

void RoomAnalyzer::get_cluster_endpoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& skeleton_cloud, pcl::PointXY& p1, pcl::PointXY& p2) {
  pcl::PointXYZRGB min, max;
  pcl::getMinMax3D(*skeleton_cloud, min, max);
  p1.x = min.x;
  p1.y = min.y;
  p2.x = max.x;
  p2.y = max.y;
}

bool RoomAnalyzer::get_centroid_location(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& skeleton_cloud, const pcl::PointXY& p1, const pcl::PointXY& p2) {
  pcl::PointXYZRGB min, max;
  min.x = p1.x;
  min.y = p1.y;
  max.x = p2.x;
  max.y = p2.y;

  pcl::PointXYZRGB centroid = compute_centroid(min, max);

  // get the dist of centroid wrt to all the points in the cluster
  // centroids lying outside will have higher distance to the points in the cluster
  float min_dist = 100;
  for(size_t i = 0; i < skeleton_cloud->points.size(); ++i) {
    float dist = sqrt(pow(centroid.x - skeleton_cloud->points[i].x, 2) + pow(centroid.y - skeleton_cloud->points[i].y, 2));

    if(dist < min_dist) {
      min_dist = dist;
    }
  }

  if(min_dist > 0.5) {
    std::cout << "centroid outside the cluster! Do something " << std::endl;
    return false;
  }

  return true;
}

bool RoomAnalyzer::get_centroid_location(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& skeleton_cloud, const geometry_msgs::Point& room_center) {
  // get the dist of centroid wrt to all the points in the cluster
  // centroids lying outside will have higher distance to the points in the cluster
  float min_dist = 100;
  for(size_t i = 0; i < skeleton_cloud->points.size(); ++i) {
    float dist = sqrt(pow(room_center.x - skeleton_cloud->points[i].x, 2) + pow(room_center.y - skeleton_cloud->points[i].y, 2));

    if(dist < min_dist) {
      min_dist = dist;
    }
  }

  if(min_dist > 0.5) {
    std::cout << "centroid outside the cluster! Discarding the found room " << std::endl;
    return false;
  }

  return true;
}

pcl::PointXYZRGB RoomAnalyzer::compute_centroid(const pcl::PointXYZRGB& p1, const pcl::PointXYZRGB& p2) {
  pcl::PointXYZRGB center;
  if(fabs(p1.x) > fabs(p2.x)) {
    float size = p1.x - p2.x;
    center.x = (size / 2) + p2.x;
  } else {
    float size = p2.x - p1.x;
    center.x = (size / 2) + p1.x;
  }

  if(fabs(p1.y) > fabs(p2.y)) {
    float size = p1.y - p2.y;
    center.y = (size / 2) + p2.y;
  } else {
    float size = p2.y - p1.y;
    center.y = (size / 2) + p1.y;
  }

  center.z = p1.z;
  return center;
}

void RoomAnalyzer::get_convex_hull(pcl::PointCloud<pcl::PointXYZRGB>::Ptr skeleton_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_hull, float& area) {
  // Create a convex hull representation of the projected inliers
  pcl::ConvexHull<pcl::PointXYZRGB> convex_hull;

  std::vector<pcl::Vertices> polygons;
  convex_hull.setInputCloud(skeleton_cloud);
  // chull.setAlpha (0.1);
  convex_hull.setDimension(2);
  convex_hull.setComputeAreaVolume(true);
  convex_hull.reconstruct(*cloud_hull, polygons);
  // std::cout << "polygons before: " << polygons[0].vertices.size() << std::endl;
  area = convex_hull.getTotalArea();

  return;
}

bool RoomAnalyzer::perform_room_segmentation(const std::vector<s_graphs::PlaneData>& current_x_vert_planes, const std::vector<s_graphs::PlaneData>& current_y_vert_planes, int& room_cluster_counter, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull, std::vector<s_graphs::RoomData>& room_candidates_vec, std::vector<std::pair<int, int>> connected_subgraph_map) {
  pcl::PointXY p1;
  pcl::PointXY p2;
  get_cluster_endpoints(cloud_cluster, p1, p2);
  // bool centroid_inside = room_analyzer->get_centroid_location(cloud_cluster, p1, p2);
  float room_width_threshold = 1.0;

  // std::cout << "cluster " << room_cluster_counter << " has x min and x max: " << p1.x << ", " << p2.x << std::endl;
  // std::cout << "cluster " << room_cluster_counter << " has y min and y max: " << p1.y << ", " << p2.y << std::endl;
  geometry_msgs::Point room_length = get_room_length(p1, p2);

  // TODO:HB check room width here
  if(room_length.x < 0.5 || room_length.y < 0.5) {
    room_cluster_counter++;
    return false;
  } else {
    // check how many planes are extracted
    // if four planes are found its a bounded room
    // if 2 parallel planes are found it an ifnite corridor
    bool found_x1_plane = false;
    bool found_x2_plane = false;
    bool found_y1_plane = false;
    bool found_y2_plane = false;
    s_graphs::PlaneData x_plane1;
    x_plane1.nx = 0;
    x_plane1.ny = 0;
    x_plane1.nz = 0;
    s_graphs::PlaneData x_plane2;
    x_plane2.nx = 0;
    x_plane2.ny = 0;
    x_plane2.nz = 0;
    s_graphs::PlaneData y_plane1;
    y_plane1.nx = 0;
    y_plane1.ny = 0;
    y_plane1.nz = 0;
    s_graphs::PlaneData y_plane2;
    y_plane2.nx = 0;
    y_plane2.ny = 0;
    y_plane1.nz = 0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sub_cloud_hull = get_room_planes(current_x_vert_planes, current_y_vert_planes, p1, p2, cloud_cluster, x_plane1, x_plane2, y_plane1, y_plane2, found_x1_plane, found_x2_plane, found_y1_plane, found_y2_plane);

    // if found all four planes its a room
    if(found_x1_plane && found_x2_plane && found_y1_plane && found_y2_plane) {
      plane_utils->correct_plane_d(PlaneUtils::plane_class::X_VERT_PLANE, x_plane1);
      plane_utils->correct_plane_d(PlaneUtils::plane_class::X_VERT_PLANE, x_plane2);
      plane_utils->correct_plane_d(PlaneUtils::plane_class::Y_VERT_PLANE, y_plane1);
      plane_utils->correct_plane_d(PlaneUtils::plane_class::Y_VERT_PLANE, y_plane2);

      // first check the width of the rooms
      float x_plane_width = plane_utils->width_between_planes(x_plane1, x_plane2);
      float y_plane_width = plane_utils->width_between_planes(y_plane1, y_plane2);

      if(x_plane_width < room_width_threshold || y_plane_width < room_width_threshold) {
        std::cout << "returning as the room is not sufficiently wide" << std::endl;
        return false;
      }

      if(!check_x1yplane_alignment(x_plane1.plane_points, y_plane1.plane_points) || !check_x1yplane_alignment(x_plane1.plane_points, y_plane2.plane_points)) {
        std::cout << "returning as not a valid room configuration" << std::endl;
        return false;
      }

      if(!check_x2yplane_alignment(x_plane2.plane_points, y_plane1.plane_points) || !check_x2yplane_alignment(x_plane2.plane_points, y_plane2.plane_points)) {
        std::cout << "returning as not a valid room configuration" << std::endl;
        return false;
      }

      if(!check_y1xplane_alignment(y_plane1.plane_points, x_plane1.plane_points) || !check_y1xplane_alignment(y_plane1.plane_points, x_plane2.plane_points)) {
        std::cout << "returning as not a valid room configuration" << std::endl;
        return false;
      }

      if(!check_y2xplane_alignment(y_plane2.plane_points, x_plane1.plane_points) || !check_y2xplane_alignment(y_plane2.plane_points, x_plane2.plane_points)) {
        std::cout << "returning as not a valid room configuration" << std::endl;
        return false;
      }
      geometry_msgs::Point room_center = get_room_center(x_plane1, x_plane2, y_plane1, y_plane2);
      bool centroid_inside = get_centroid_location(cloud_cluster, room_center);
      if(!centroid_inside) {
        std::cout << "returning as the room center is outside the cluster" << std::endl;
        return false;
      }

      std::vector<int> neighbour_ids;
      for(const auto& connected_ids : connected_subgraph_map) {
        if(connected_ids.first == cloud_cluster->header.seq)
          neighbour_ids.push_back(connected_ids.second);
        else if(connected_ids.second == cloud_cluster->header.seq)
          neighbour_ids.push_back(connected_ids.first);
      }

      // clear plane points which are not required now
      x_plane1.plane_points.clear();
      x_plane2.plane_points.clear();
      y_plane1.plane_points.clear();
      y_plane2.plane_points.clear();

      s_graphs::RoomData room_candidate;
      room_candidate.id = cloud_cluster->header.seq;
      room_candidate.neighbour_ids = neighbour_ids;
      room_candidate.room_length = room_length;
      room_candidate.room_center = room_center;
      room_candidate.x_planes.push_back(x_plane1);
      room_candidate.x_planes.push_back(x_plane2);
      room_candidate.y_planes.push_back(y_plane1);
      room_candidate.y_planes.push_back(y_plane2);
      room_candidates_vec.push_back(room_candidate);
      room_cluster_counter++;
      return true;
    }
    // if found only two x planes are found at x corridor
    else if(found_x1_plane && found_x2_plane && (!found_y1_plane || !found_y2_plane)) {
      if(sub_cloud_hull->points.size() > 0) get_cluster_endpoints(sub_cloud_hull, p1, p2);

      plane_utils->correct_plane_d(PlaneUtils::plane_class::X_VERT_PLANE, x_plane1);
      plane_utils->correct_plane_d(PlaneUtils::plane_class::X_VERT_PLANE, x_plane2);

      float x_plane_width = plane_utils->width_between_planes(x_plane1, x_plane2);
      if(x_plane_width < room_width_threshold) {
        std::cout << "returning as the room is not sufficiently wide" << std::endl;
        return false;
      }

      geometry_msgs::Point room_center = get_corridor_center(PlaneUtils::plane_class::X_VERT_PLANE, p1, p2, x_plane1, x_plane2);
      bool centroid_inside = get_centroid_location(cloud_cluster, room_center);
      if(!centroid_inside) {
        std::cout << "returning as the room center is outside the cluster" << std::endl;
        return false;
      }

      std::vector<int> neighbour_ids;
      for(const auto& connected_ids : connected_subgraph_map) {
        if(connected_ids.first == cloud_cluster->header.seq)
          neighbour_ids.push_back(connected_ids.second);
        else if(connected_ids.second == cloud_cluster->header.seq)
          neighbour_ids.push_back(connected_ids.first);
      }

      // clear plane points which are not required now
      x_plane1.plane_points.clear();
      x_plane2.plane_points.clear();
      y_plane1.plane_points.clear();
      y_plane2.plane_points.clear();

      s_graphs::RoomData room_candidate;
      room_candidate.id = cloud_cluster->header.seq;
      room_candidate.neighbour_ids = neighbour_ids;
      room_candidate.room_length = room_length;
      room_candidate.room_center = room_center;
      room_candidate.x_planes.push_back(x_plane1);
      room_candidate.x_planes.push_back(x_plane2);
      room_candidates_vec.push_back(room_candidate);
      room_cluster_counter++;
      return true;
    }
    // if found only two y planes are found at y corridor
    else if(found_y1_plane && found_y2_plane && (!found_x1_plane || !found_x2_plane)) {
      if(sub_cloud_hull->points.size() > 0) get_cluster_endpoints(sub_cloud_hull, p1, p2);

      plane_utils->correct_plane_d(PlaneUtils::plane_class::Y_VERT_PLANE, y_plane1);
      plane_utils->correct_plane_d(PlaneUtils::plane_class::Y_VERT_PLANE, y_plane2);

      float y_plane_width = plane_utils->width_between_planes(y_plane1, y_plane2);
      if(y_plane_width < room_width_threshold) {
        std::cout << "returning as the room is not sufficiently wide" << std::endl;
        return false;
      }

      geometry_msgs::Point room_center = get_corridor_center(PlaneUtils::plane_class::Y_VERT_PLANE, p1, p2, y_plane1, y_plane2);
      bool centroid_inside = get_centroid_location(cloud_cluster, room_center);
      if(!centroid_inside) {
        std::cout << "returning as the room center is outside the cluster" << std::endl;
        return false;
      }

      std::vector<int> neighbour_ids;
      for(const auto& connected_ids : connected_subgraph_map) {
        if(connected_ids.first == cloud_cluster->header.seq)
          neighbour_ids.push_back(connected_ids.second);
        else if(connected_ids.second == cloud_cluster->header.seq)
          neighbour_ids.push_back(connected_ids.first);
      }

      // clear plane points which are not required now
      x_plane1.plane_points.clear();
      x_plane2.plane_points.clear();
      y_plane1.plane_points.clear();
      y_plane2.plane_points.clear();

      s_graphs::RoomData room_candidate;
      room_candidate.id = cloud_cluster->header.seq;
      room_candidate.neighbour_ids = neighbour_ids;
      room_candidate.room_length = room_length;
      room_candidate.room_center = room_center;
      room_candidate.y_planes.push_back(y_plane1);
      room_candidate.y_planes.push_back(y_plane2);
      room_candidates_vec.push_back(room_candidate);
      return true;
    } else {
      room_cluster_counter++;
      return false;
    }
  }
  return false;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RoomAnalyzer::get_room_planes(const std::vector<s_graphs::PlaneData>& current_x_vert_planes, const std::vector<s_graphs::PlaneData>& current_y_vert_planes, pcl::PointXY p_min, pcl::PointXY p_max, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull, s_graphs::PlaneData& x_plane1, s_graphs::PlaneData& x_plane2, s_graphs::PlaneData& y_plane1, s_graphs::PlaneData& y_plane2, bool& found_x1_plane, bool& found_x2_plane, bool& found_y1_plane, bool& found_y2_plane) {
  float room_dist_thres = 1.5;
  float plane_point_dist_thres = 1.5;
  float min_dist_x1 = 100;
  float min_dist_x2 = 100;
  std::vector<float> min_x1_plane_points_dist;
  min_x1_plane_points_dist.resize(2);
  min_x1_plane_points_dist[0] = 100;
  min_x1_plane_points_dist[1] = 100;
  std::vector<float> min_x2_plane_points_dist;
  min_x2_plane_points_dist.resize(2);
  min_x2_plane_points_dist[0] = 100;
  min_x2_plane_points_dist[1] = 100;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull_x1(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull_x2(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull_y1(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull_y2(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr sub_cloud_hull(new pcl::PointCloud<pcl::PointXYZRGB>);

  bool use_max_neighbours_algo = true;
  int max_x1_neighbours, max_x2_neighbours, max_y1_neighbours, max_y2_neighbours;
  max_x1_neighbours = max_x2_neighbours = max_y1_neighbours = max_y2_neighbours = 0;
  int min_neighbors_thres = 5;

  pcl::PointXY top_right, bottom_right, top_left, bottom_left;
  if(!use_max_neighbours_algo) {
    if(!compute_diagonal_points(cloud_hull, p_min, p_max, top_right, bottom_right, top_left, bottom_left)) {
      std::cout << "didnt not get diagonal points" << std::endl;
      return sub_cloud_hull;
    }
  }

  int point_cloud_size_thres = 500;
  if(cloud_hull->points.size() > point_cloud_size_thres) {
    downsample_cloud(cloud_hull);
  }

  for(const auto& x_plane : current_x_vert_planes) {
    if(x_plane.nx < 0) {
      continue;
    }

    // std::cout << "xplane1: " << x_plane.nx << ", " << x_plane.ny << ", " << x_plane.nz << ", " << x_plane.d << std::endl;
    float dist_x1 = -1 * (x_plane.nx * p_min.x + x_plane.ny * p_min.y);
    float diff_dist_x1 = 100;
    diff_dist_x1 = sqrt((dist_x1 - x_plane.d) * (dist_x1 - x_plane.d));

    // std::cout << "diff dist x1: " << diff_dist_x1 << std::endl;
    if(use_max_neighbours_algo) {
      int x1_neighbours = find_plane_points(cloud_hull, x_plane, cloud_hull_x1);

      if(x1_neighbours > max_x1_neighbours) {
        min_dist_x1 = diff_dist_x1;
        max_x1_neighbours = x1_neighbours;
        x_plane1 = x_plane;
      }
    } else {
      std::vector<float> diff_x_plane_points_dist = find_plane_points(bottom_left, bottom_right, x_plane);
      if(diff_x_plane_points_dist[0] < min_x1_plane_points_dist[0] && diff_x_plane_points_dist[1] < min_x1_plane_points_dist[1]) {
        min_dist_x1 = diff_dist_x1;
        min_x1_plane_points_dist[0] = diff_x_plane_points_dist[0];
        min_x1_plane_points_dist[1] = diff_x_plane_points_dist[1];

        x_plane1 = x_plane;
      }
    }
  }

  for(const auto& x_plane : current_x_vert_planes) {
    if(x_plane.nx > 0) {
      continue;
    }

    // std::cout << "xplane2: " << x_plane.nx << ", " << x_plane.ny << ", " << x_plane.nz << ", " << x_plane.d << std::endl;
    float dist_x2 = -1 * (x_plane.nx * p_max.x + x_plane.ny * p_max.y);
    float diff_dist_x2 = 100;
    diff_dist_x2 = sqrt((dist_x2 - x_plane.d) * (dist_x2 - x_plane.d));

    // std::cout << "diff dist x2: " << diff_dist_x2 << std::endl;
    if(use_max_neighbours_algo) {
      int x2_neighbours = find_plane_points(cloud_hull, x_plane, cloud_hull_x2);

      if(x2_neighbours > max_x2_neighbours) {
        min_dist_x2 = diff_dist_x2;
        max_x2_neighbours = x2_neighbours;
        x_plane2 = x_plane;
      }
    } else {
      std::vector<float> diff_x_plane_points_dist = find_plane_points(top_right, top_left, x_plane);
      if(diff_x_plane_points_dist[0] < min_x2_plane_points_dist[0] && diff_x_plane_points_dist[1] < min_x2_plane_points_dist[1]) {
        min_dist_x2 = diff_dist_x2;
        min_x2_plane_points_dist[0] = diff_x_plane_points_dist[0];
        min_x2_plane_points_dist[1] = diff_x_plane_points_dist[1];

        x_plane2 = x_plane;
      }
    }
  }

  // std::cout << "selected xplane1 : " << x_plane1.nx << ", " << x_plane1.ny << ", " << x_plane1.nz << ", " << x_plane1.d << std::endl;
  // std::cout << "min_dist_x1 : " << min_dist_x1 << std::endl;
  // std::cout << "min_x1_plane_points_dist[0] : " << min_x1_plane_points_dist[0] << std::endl;
  // std::cout << "min_x1_plane_points_dist[1] : " << min_x1_plane_points_dist[1] << std::endl;

  // std::cout << "selected xplane2 : " << x_plane2.nx << ", " << x_plane2.ny << ", " << x_plane2.nz << ", " << x_plane2.d << std::endl;
  // std::cout << "min_dist_x2 : " << min_dist_x2 << std::endl;
  // std::cout << "min_x2_plane_points_dist[0] : " << min_x2_plane_points_dist[0] << std::endl;
  // std::cout << "min_x2_plane_points_dist[1] : " << min_x2_plane_points_dist[1] << std::endl;

  if(x_plane1.nx * x_plane2.nx >= 0) {
    // std::cout << "no xplane1 found " << std::endl;
    // std::cout << "no xplane2 found " << std::endl;
    found_x1_plane = false, found_x2_plane = false;
  } else {
    if(use_max_neighbours_algo) {
      bool planes_placed_correctly = false;
      if(!x_plane1.plane_points.empty() && !x_plane2.plane_points.empty()) {
        planes_placed_correctly = plane_utils->compute_point_difference(x_plane1.plane_points.back().x, x_plane2.plane_points.back().x);
      }

      if(max_x1_neighbours >= min_neighbors_thres && planes_placed_correctly) {
        // std::cout << "room has xplane1: " << x_plane1.nx << ", " << x_plane1.ny << ", " << x_plane1.nz << ", " << x_plane1.d << std::endl;
        found_x1_plane = true;
      } else {
        // std::cout << "no xplane1 found " << std::endl;
        found_x1_plane = false;
      }
      if(max_x2_neighbours >= min_neighbors_thres && planes_placed_correctly) {
        // std::cout << "room has xplane2: " << x_plane2.nx << ", " << x_plane2.ny << ", " << x_plane2.nz << ", " << x_plane2.d << std::endl;
        found_x2_plane = true;
      } else {
        // std::cout << "no xplane2 found " << std::endl;
        found_x2_plane = false;
      }
    } else {
      if(min_dist_x1 < room_dist_thres && min_x1_plane_points_dist[0] < plane_point_dist_thres && min_x1_plane_points_dist[1] < plane_point_dist_thres) {
        // std::cout << "room has xplane1: " << x_plane1.nx << ", " << x_plane1.ny << ", " << x_plane1.nz << ", " << x_plane1.d << std::endl;
        found_x1_plane = true;
      } else {
        // std::cout << "no xplane1 found " << std::endl;
        found_x1_plane = false;
      }
      if(min_dist_x2 < room_dist_thres && min_x2_plane_points_dist[0] < plane_point_dist_thres && min_x2_plane_points_dist[1] < plane_point_dist_thres) {
        // std::cout << "room has xplane2: " << x_plane2.nx << ", " << x_plane2.ny << ", " << x_plane2.nz << ", " << x_plane2.d << std::endl;
        found_x2_plane = true;
      } else {
        // std::cout << "no xplane2 found " << std::endl;
        found_x2_plane = false;
      }
    }
  }

  float min_dist_y1 = 100;
  float min_dist_y2 = 100;
  std::vector<float> min_y1_plane_points_dist;
  min_y1_plane_points_dist.resize(2);
  min_y1_plane_points_dist[0] = 100;
  min_y1_plane_points_dist[1] = 100;
  std::vector<float> min_y2_plane_points_dist;
  min_y2_plane_points_dist.resize(2);
  min_y2_plane_points_dist[0] = 100;
  min_y2_plane_points_dist[1] = 100;

  for(const auto& y_plane : current_y_vert_planes) {
    if(y_plane.ny < 0) {
      continue;
    }

    // std::cout << "y_plane1: " << y_plane.nx << ", " << y_plane.ny << ", " << y_plane.nz << ", " << y_plane.d << std::endl;
    float dist_y1 = -1 * (y_plane.nx * p_min.x + y_plane.ny * p_min.y);
    float diff_dist_y1 = 100;
    diff_dist_y1 = sqrt((dist_y1 - y_plane.d) * (dist_y1 - y_plane.d));

    if(use_max_neighbours_algo) {
      int y1_neighbours = find_plane_points(cloud_hull, y_plane, cloud_hull_y1);

      if(y1_neighbours > max_y1_neighbours) {
        min_dist_y1 = diff_dist_y1;
        max_y1_neighbours = y1_neighbours;
        y_plane1 = y_plane;
      }
    } else {
      // std::cout << "diff dist y1: " << diff_dist_y1 << std::endl;
      std::vector<float> diff_plane_points_dist = find_plane_points(top_right, bottom_right, y_plane);
      if(diff_plane_points_dist[0] < min_y1_plane_points_dist[0] && diff_plane_points_dist[1] < min_y1_plane_points_dist[1]) {
        min_dist_y1 = diff_dist_y1;
        min_y1_plane_points_dist[0] = diff_plane_points_dist[0];
        min_y1_plane_points_dist[1] = diff_plane_points_dist[1];
        y_plane1 = y_plane;
      }
    }
  }

  for(const auto& y_plane : current_y_vert_planes) {
    if(y_plane.ny > 0) {
      continue;
    }

    // std::cout << "y_plane2: " << y_plane.nx << ", " << y_plane.ny << ", " << y_plane.nz << ", " << y_plane.d << std::endl;
    float dist_y2 = -1 * (y_plane.nx * p_max.x + y_plane.ny * p_max.y);
    float diff_dist_y2 = 100;
    diff_dist_y2 = sqrt((dist_y2 - y_plane.d) * (dist_y2 - y_plane.d));

    if(use_max_neighbours_algo) {
      int y2_neighbours = find_plane_points(cloud_hull, y_plane, cloud_hull_y2);

      if(y2_neighbours > max_y2_neighbours) {
        min_dist_y2 = diff_dist_y2;
        max_y2_neighbours = y2_neighbours;
        y_plane2 = y_plane;
      }
    } else {
      // std::cout << "diff dist y2: " << diff_dist_y2 << std::endl;
      std::vector<float> diff_plane_points_dist = find_plane_points(bottom_left, top_left, y_plane);
      if(diff_plane_points_dist[0] < min_y2_plane_points_dist[0] && diff_plane_points_dist[1] < min_y2_plane_points_dist[1]) {
        min_dist_y2 = diff_dist_y2;
        min_y2_plane_points_dist[0] = diff_plane_points_dist[0];
        min_y2_plane_points_dist[1] = diff_plane_points_dist[1];
        y_plane2 = y_plane;
      }
    }
  }

  // std::cout << "selected yplane1 : " << y_plane1.nx << ", " << y_plane1.ny << ", " << y_plane1.nz << ", " << y_plane1.d << std::endl;
  // std::cout << "min_dist_y1 : " << min_dist_y1 << std::endl;
  // std::cout << "min_y1_plane_points_dist[0] : " << min_y1_plane_points_dist[0] << std::endl;
  // std::cout << "min_y1_plane_points_dist[1] : " << min_y1_plane_points_dist[1] << std::endl;

  // std::cout << "selected yplane2 : " << y_plane2.nx << ", " << y_plane2.ny << ", " << y_plane2.nz << ", " << y_plane2.d << std::endl;
  // std::cout << "min_dist_y2 : " << min_dist_y2 << std::endl;
  // std::cout << "min_y2_plane_points_dist[0] : " << min_y2_plane_points_dist[0] << std::endl;
  // std::cout << "min_y2_plane_points_dist[1] : " << min_y2_plane_points_dist[1] << std::endl;

  if(y_plane1.ny * y_plane2.ny >= 0) {
    // std::cout << "no yplane1 found " << std::endl;
    // std::cout << "no yplane2 found " << std::endl;
    found_y1_plane = false, found_y2_plane = false;
  } else {
    if(use_max_neighbours_algo) {
      bool planes_placed_correctly = false;
      if(!y_plane1.plane_points.empty() && !y_plane2.plane_points.empty()) {
        planes_placed_correctly = plane_utils->compute_point_difference(y_plane1.plane_points.back().y, y_plane2.plane_points.back().y);
      }

      if(max_y1_neighbours >= min_neighbors_thres && planes_placed_correctly) {
        // std::cout << "room has yplane1: " << y_plane1.nx << ", " << y_plane1.ny << ", " << y_plane1.nz << ", " << y_plane1.d << std::endl;
        found_y1_plane = true;
      } else {
        // std::cout << "no yplane1 found " << std::endl;
        found_y1_plane = false;
      }
      if(max_y2_neighbours >= min_neighbors_thres && planes_placed_correctly) {
        // std::cout << "room has yplane2: " << y_plane2.nx << ", " << y_plane2.ny << ", " << y_plane2.nz << ", " << y_plane2.d << std::endl;
        found_y2_plane = true;
      } else {
        // std::cout << "no yplane2 found " << std::endl;
        found_y2_plane = false;
      }
    } else {
      if(min_dist_y1 < room_dist_thres && min_y1_plane_points_dist[0] < plane_point_dist_thres && min_y1_plane_points_dist[1] < plane_point_dist_thres) {
        // std::cout << "room has yplane1: " << y_plane1.nx << ", " << y_plane1.ny << ", " << y_plane1.nz << ", " << y_plane1.d << std::endl;
        found_y1_plane = true;
      } else {
        // std::cout << "no yplane1 found " << std::endl;
        found_y1_plane = false;
      }
      if(min_dist_y2 < room_dist_thres && min_y2_plane_points_dist[0] < plane_point_dist_thres && min_y2_plane_points_dist[1] < plane_point_dist_thres) {
        // std::cout << "room has yplane2: " << y_plane2.nx << ", " << y_plane2.ny << ", " << y_plane2.nz << ", " << y_plane2.d << std::endl;
        found_y2_plane = true;
      } else {
        // std::cout << "no yplane2 found " << std::endl;
        found_y2_plane = false;
      }
    }
  }

  if(found_x1_plane && found_x2_plane && found_y1_plane && found_y2_plane) {
    for(int i = 0; i < cloud_hull_x1->points.size(); ++i) sub_cloud_hull->points.push_back(cloud_hull_x1->points[i]);
    for(int i = 0; i < cloud_hull_x2->points.size(); ++i) sub_cloud_hull->points.push_back(cloud_hull_x2->points[i]);
    for(int i = 0; i < cloud_hull_y1->points.size(); ++i) sub_cloud_hull->points.push_back(cloud_hull_y1->points[i]);
    for(int i = 0; i < cloud_hull_y2->points.size(); ++i) sub_cloud_hull->points.push_back(cloud_hull_y2->points[i]);
  } else if(found_x1_plane && found_x2_plane && (!found_y1_plane || !found_y2_plane)) {
    for(int i = 0; i < cloud_hull_x1->points.size(); ++i) sub_cloud_hull->points.push_back(cloud_hull_x1->points[i]);
    for(int i = 0; i < cloud_hull_x2->points.size(); ++i) sub_cloud_hull->points.push_back(cloud_hull_x2->points[i]);
  } else if(found_y1_plane && found_y2_plane && (!found_y1_plane || !found_y2_plane)) {
    for(int i = 0; i < cloud_hull_y1->points.size(); ++i) sub_cloud_hull->points.push_back(cloud_hull_y1->points[i]);
    for(int i = 0; i < cloud_hull_y2->points.size(); ++i) sub_cloud_hull->points.push_back(cloud_hull_y2->points[i]);
  }

  return sub_cloud_hull;
}

bool RoomAnalyzer::check_x1yplane_alignment(const std::vector<geometry_msgs::Vector3> x_plane1_points, const std::vector<geometry_msgs::Vector3> y_plane_points) {
  bool valid_room_config = false;
  int point_count = 0;
  for(int i = 0; i < y_plane_points.size(); ++i) {
    for(int j = 0; j < x_plane1_points.size(); ++j) {
      float dist = y_plane_points[i].x - x_plane1_points[j].x;
      if(dist > 0) {
        point_count++;
        break;
      }
    }
    if(point_count > 500) {
      valid_room_config = true;
      break;
    }
  }
  return valid_room_config;
}

bool RoomAnalyzer::check_x2yplane_alignment(const std::vector<geometry_msgs::Vector3> x_plane2_points, const std::vector<geometry_msgs::Vector3> y_plane_points) {
  bool valid_room_config = false;
  int point_count = 0;
  for(int i = 0; i < y_plane_points.size(); ++i) {
    for(int j = 0; j < x_plane2_points.size(); ++j) {
      float dist = y_plane_points[i].x - x_plane2_points[j].x;
      if(dist < 0) {
        point_count++;
        break;
      }
    }
    if(point_count > 500) {
      valid_room_config = true;
      break;
    }
  }
  return valid_room_config;
}

bool RoomAnalyzer::check_y1xplane_alignment(const std::vector<geometry_msgs::Vector3> y_plane1_points, const std::vector<geometry_msgs::Vector3> x_plane_points) {
  bool valid_room_config = false;
  int point_count = 0;
  for(int i = 0; i < x_plane_points.size(); ++i) {
    for(int j = 0; j < y_plane1_points.size(); ++j) {
      float dist = x_plane_points[i].y - y_plane1_points[j].y;
      if(dist > 0) {
        point_count++;
        break;
      }
    }
    if(point_count > 500) {
      valid_room_config = true;
      break;
    }
  }
  return valid_room_config;
}

bool RoomAnalyzer::check_y2xplane_alignment(const std::vector<geometry_msgs::Vector3> y_plane2_points, const std::vector<geometry_msgs::Vector3> x_plane_points) {
  bool valid_room_config = false;
  int point_count = 0;
  for(int i = 0; i < x_plane_points.size(); ++i) {
    for(int j = 0; j < y_plane2_points.size(); ++j) {
      float dist = x_plane_points[i].y - y_plane2_points[j].y;
      if(dist < 0) {
        point_count++;
        break;
      }
    }
    if(point_count > 500) {
      valid_room_config = true;
      break;
    }
  }
  return valid_room_config;
}

bool RoomAnalyzer::compute_diagonal_points(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_hull, const pcl::PointXY min, const pcl::PointXY max, pcl::PointXY& top_right, pcl::PointXY& bottom_right, pcl::PointXY& top_left, pcl::PointXY& bottom_left) {
  // get the set of diagonal points
  pcl::PointXY bottom_right_img, top_right_img, bottom_left_img, top_left_img;
  bottom_right_img.x = min.x;
  bottom_right_img.y = min.y;
  top_left_img.x = max.x;
  top_left_img.y = max.y;

  // this is imaginary point and based on this we will find the closest point in our hull
  bottom_left_img.x = min.x;
  bottom_left_img.y = max.y;
  top_right_img.x = max.x;
  top_right_img.y = min.y;

  // find the neareast neighbour to our imaginary points in our hull
  float min_dist_top_right = 100;
  float min_dist_bottom_right = 100;
  float min_dist_top_left = 100;
  float min_dist_bottom_left = 100;

  int top_right_index = -1;
  int bottom_right_index = -1;
  int top_left_index = -1;
  int bottom_left_index = -1;

  for(int i = 0; i < cloud_hull->points.size(); ++i) {
    float dist_top_right = sqrt(pow(cloud_hull->points[i].x - top_right_img.x, 2) + pow(cloud_hull->points[i].y - top_right_img.y, 2));
    if(dist_top_right < min_dist_top_right) {
      min_dist_top_right = dist_top_right;
      top_right_index = i;
    }

    float dist_bottom_right = sqrt(pow(cloud_hull->points[i].x - bottom_right_img.x, 2) + pow(cloud_hull->points[i].y - bottom_right_img.y, 2));
    if(dist_bottom_right < min_dist_bottom_right) {
      min_dist_bottom_right = dist_bottom_right;
      bottom_right_index = i;
    }

    float dist_top_left = sqrt(pow(cloud_hull->points[i].x - top_left_img.x, 2) + pow(cloud_hull->points[i].y - top_left_img.y, 2));
    if(dist_top_left < min_dist_top_left) {
      min_dist_top_left = dist_top_left;
      top_left_index = i;
    }

    float dist_bottom_left = sqrt(pow(cloud_hull->points[i].x - bottom_left_img.x, 2) + pow(cloud_hull->points[i].y - bottom_left_img.y, 2));
    if(dist_bottom_left < min_dist_bottom_left) {
      min_dist_bottom_left = dist_bottom_left;
      bottom_left_index = i;
    }
  }

  if(top_right_index != -1 && bottom_right_index != -1 && top_left_index != -1 && bottom_left_index != -1) {
    // if(min_dist_top_right < 1.0 && min_dist_bottom_left < 1.0) {
    top_right.x = cloud_hull->points[top_right_index].x;
    top_right.y = cloud_hull->points[top_right_index].y;

    bottom_right.x = cloud_hull->points[bottom_right_index].x;
    bottom_right.y = cloud_hull->points[bottom_right_index].y;

    top_left.x = cloud_hull->points[top_left_index].x;
    top_left.y = cloud_hull->points[top_left_index].y;

    bottom_left.x = cloud_hull->points[bottom_left_index].x;
    bottom_left.y = cloud_hull->points[bottom_left_index].y;

    return true;
    //}
  }
  return false;
}

std::vector<float> RoomAnalyzer::find_plane_points(const pcl::PointXY& start_point, const pcl::PointXY& end_point, const s_graphs::PlaneData& plane) {
  float min_start_point_plane_dist = 100;
  float min_end_point_plane_dist = 100;
  std::vector<float> plane_point_distances;
  geometry_msgs::Vector3 closest_start_plane_point, closest_end_plane_point;
  for(const auto& plane_point : plane.plane_points) {
    float start_plane_point_dist = sqrt(pow(start_point.x - plane_point.x, 2) + pow(start_point.y - plane_point.y, 2));

    float end_plane_point_dist = sqrt(pow(end_point.x - plane_point.x, 2) + pow(end_point.y - plane_point.y, 2));

    if(start_plane_point_dist < min_start_point_plane_dist) {
      min_start_point_plane_dist = start_plane_point_dist;
      closest_start_plane_point = plane_point;
    }

    if(end_plane_point_dist < min_end_point_plane_dist) {
      min_end_point_plane_dist = end_plane_point_dist;
      closest_end_plane_point = plane_point;
    }
  }

  // std::cout << "closest_start_plane_point " << closest_start_plane_point.x << " ; " << closest_start_plane_point.y << std::endl;
  // std::cout << "closest_end_plane_point " << closest_end_plane_point.x << " ; " << closest_end_plane_point.y << std::endl;
  plane_point_distances.push_back(min_start_point_plane_dist);
  plane_point_distances.push_back(min_end_point_plane_dist);

  return plane_point_distances;
}

void RoomAnalyzer::downsample_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_hull) {
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud_hull);
  sor.setLeafSize(0.1f, 0.1f, 0.1f);
  sor.filter(*cloud_hull);
}

int RoomAnalyzer::find_plane_points(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_hull, const s_graphs::PlaneData& plane, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& sub_cloud_hull) {
  int num_neighbours = 0;
  double point_hull_dist_thres = 1.0;

  for(int i = 0; i < cloud_hull->points.size(); ++i) {
    double min_point_hull_dist = 1000;
    for(const auto& plane_point : plane.plane_points) {
      float point_hull_dist = sqrt(pow(plane_point.x - cloud_hull->points[i].x, 2) + pow(plane_point.y - cloud_hull->points[i].y, 2));
      if(point_hull_dist < min_point_hull_dist) {
        min_point_hull_dist = point_hull_dist;
      }
    }
    if(min_point_hull_dist < point_hull_dist_thres) {
      num_neighbours++;
      sub_cloud_hull->points.push_back(cloud_hull->points[i]);
    }
  }
  // std::cout << "num nieghbours: " << num_neighbours << std::endl;
  return num_neighbours;
}

}  // namespace s_graphs
