// SPDX-License-Identifier: BSD-2-Clause

#include <s_graphs/room_analyzer.hpp>

namespace s_graphs {

RoomAnalyzer::RoomAnalyzer(const ros::NodeHandle& private_nh) {
  nh = private_nh;
  cloud_clusters_.clear();
  connected_subgraphs_.clear();
}

RoomAnalyzer::~RoomAnalyzer() {
  cloud_clusters_.clear();
  connected_subgraphs_.clear();
}

void RoomAnalyzer::analyze_skeleton_graph(const visualization_msgs::MarkerArray::Ptr& skeleton_graph_msg) {
  cloud_clusters_.clear();
  connected_subgraphs_.clear();

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
      subgraph_id++;
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

  skeleton_graph_mutex.lock();
  cloud_clusters_ = curr_cloud_clusters;
  connected_subgraphs_ = connected_subgraph_map;
  skeleton_graph_mutex.unlock();

  return;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> RoomAnalyzer::get_cloud_clusters() {
  return cloud_clusters_;
}

std::vector<std::pair<int, int>> RoomAnalyzer::get_connected_graph() {
  return connected_subgraphs_;
}

geometry_msgs::Point RoomAnalyzer::get_room_length(pcl::PointXY p1, pcl::PointXY p2) {
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

geometry_msgs::Point RoomAnalyzer::get_room_center(pcl::PointXY p1, pcl::PointXY p2, s_graphs::PlaneData x_plane1, s_graphs::PlaneData x_plane2, s_graphs::PlaneData y_plane1, s_graphs::PlaneData y_plane2) {
  geometry_msgs::Point center;

  if(fabs(x_plane1.d) > fabs(x_plane2.d)) {
    double size = x_plane1.d - x_plane2.d;
    center.x = (((size) / 2) + x_plane2.d);
  } else {
    double size = x_plane2.d - x_plane1.d;
    center.x = (((size) / 2) + x_plane1.d);
  }

  if(fabs(y_plane1.d) > fabs(y_plane2.d)) {
    double size = y_plane1.d - y_plane2.d;
    center.y = (((size) / 2) + y_plane2.d);
  } else {
    double size = y_plane2.d - y_plane1.d;
    center.y = (((size) / 2) + y_plane1.d);
  }

  return center;
}

geometry_msgs::Point RoomAnalyzer::get_corridor_center(int plane_type, pcl::PointXY p1, pcl::PointXY p2, s_graphs::PlaneData plane1, s_graphs::PlaneData plane2) {
  geometry_msgs::Point center;

  if(plane_type == plane_class::X_VERT_PLANE) {
    if(fabs(plane1.d) > fabs(plane2.d)) {
      double size = plane1.d - plane2.d;
      center.x = (((size) / 2) + plane2.d);
    } else {
      double size = plane2.d - plane1.d;
      center.x = (((size) / 2) + plane1.d);
    }

    if(fabs(p1.y) > fabs(p2.y)) {
      float size = p1.y - p2.y;
      center.y = (size / 2) + p2.y;
    } else {
      float size = p2.y - p1.y;
      center.y = (size / 2) + p1.y;
    }
  }

  if(plane_type == plane_class::Y_VERT_PLANE) {
    if(fabs(plane1.d) > fabs(plane2.d)) {
      double size = plane1.d - plane2.d;
      center.y = (((size) / 2) + plane2.d);
    } else {
      double size = plane2.d - plane1.d;
      center.y = (((size) / 2) + plane1.d);
    }

    if(fabs(p1.x) > fabs(p2.x)) {
      float size = p1.x - p2.x;
      center.x = (size / 2) + p2.x;
    } else {
      float size = p2.x - p1.x;
      center.x = (size / 2) + p1.x;
    }
  }

  return center;
}

bool RoomAnalyzer::get_centroid_location(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& skeleton_cloud, pcl::PointXY& p1, pcl::PointXY& p2) {
  pcl::PointXYZRGB min, max;
  pcl::getMinMax3D(*skeleton_cloud, min, max);
  p1.x = min.x;
  p1.y = min.y;
  p2.x = max.x;
  p2.y = max.y;

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

float RoomAnalyzer::get_room_width(s_graphs::PlaneData& plane1, s_graphs::PlaneData& plane2) {
  float size = 0;
  float room_width_threshold = 1.0;
  if(fabs(plane1.d) > fabs(plane2.d))
    size = fabs(plane1.d - plane2.d);
  else if(fabs(plane2.d) > fabs(plane1.d))
    size = fabs(plane2.d - plane1.d);

  return size;
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

void RoomAnalyzer::get_room_planes(const std::vector<s_graphs::PlaneData>& current_x_vert_planes, const std::vector<s_graphs::PlaneData>& current_y_vert_planes, pcl::PointXY p_min, pcl::PointXY p_max, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_hull, s_graphs::PlaneData& x_plane1, s_graphs::PlaneData& x_plane2, s_graphs::PlaneData& y_plane1, s_graphs::PlaneData& y_plane2, bool& found_x1_plane, bool& found_x2_plane, bool& found_y1_plane, bool& found_y2_plane) {
  float room_dist_thres = 1.2;
  float plane_point_dist_thres = 1.2;
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

  pcl::PointXY top_right, bottom_right, top_left, bottom_left;
  if(!compute_diagonal_points(cloud_hull, p_min, p_max, top_right, bottom_right, top_left, bottom_left)) {
    std::cout << "didnt not get diagonal points" << std::endl;
    return;
  }

  for(const auto& x_plane : current_x_vert_planes) {
    if(x_plane.nx < 0) {
      continue;
    }

    std::cout << "xplane1: " << x_plane.nx << ", " << x_plane.ny << ", " << x_plane.nz << ", " << x_plane.d << std::endl;
    float dist_x1 = -1 * (x_plane.nx * p_min.x + x_plane.ny * p_min.y);
    float diff_dist_x1 = 100;
    diff_dist_x1 = sqrt((dist_x1 - x_plane.d) * (dist_x1 - x_plane.d));

    // std::cout << "diff dist x1: " << diff_dist_x1 << std::endl;

    std::vector<float> diff_x_plane_points_dist = find_plane_points(bottom_left, bottom_right, x_plane);
    if(diff_x_plane_points_dist[0] < min_x1_plane_points_dist[0] && diff_x_plane_points_dist[1] < min_x1_plane_points_dist[1]) {
      min_dist_x1 = diff_dist_x1;
      min_x1_plane_points_dist[0] = diff_x_plane_points_dist[0];
      min_x1_plane_points_dist[1] = diff_x_plane_points_dist[1];

      x_plane1 = x_plane;
    }
  }

  for(const auto& x_plane : current_x_vert_planes) {
    if(x_plane.nx > 0) {
      continue;
    }

    std::cout << "xplane2: " << x_plane.nx << ", " << x_plane.ny << ", " << x_plane.nz << ", " << x_plane.d << std::endl;
    float dist_x2 = -1 * (x_plane.nx * p_max.x + x_plane.ny * p_max.y);
    float diff_dist_x2 = 100;
    diff_dist_x2 = sqrt((dist_x2 - x_plane.d) * (dist_x2 - x_plane.d));

    // std::cout << "diff dist x2: " << diff_dist_x2 << std::endl;
    std::vector<float> diff_x_plane_points_dist = find_plane_points(top_right, top_left, x_plane);
    if(diff_x_plane_points_dist[0] < min_x2_plane_points_dist[0] && diff_x_plane_points_dist[1] < min_x2_plane_points_dist[1]) {
      min_dist_x2 = diff_dist_x2;
      min_x2_plane_points_dist[0] = diff_x_plane_points_dist[0];
      min_x2_plane_points_dist[1] = diff_x_plane_points_dist[1];

      x_plane2 = x_plane;
    }
  }

  std::cout << "selected xplane1 : " << x_plane1.nx << ", " << x_plane1.ny << ", " << x_plane1.nz << ", " << x_plane1.d << std::endl;
  std::cout << "min_dist_x1 : " << min_dist_x1 << std::endl;
  std::cout << "min_x1_plane_points_dist[0] : " << min_x1_plane_points_dist[0] << std::endl;
  std::cout << "min_x1_plane_points_dist[1] : " << min_x1_plane_points_dist[1] << std::endl;

  std::cout << "selected xplane2 : " << x_plane2.nx << ", " << x_plane2.ny << ", " << x_plane2.nz << ", " << x_plane2.d << std::endl;
  std::cout << "min_dist_x2 : " << min_dist_x2 << std::endl;
  std::cout << "min_x2_plane_points_dist[0] : " << min_x2_plane_points_dist[0] << std::endl;
  std::cout << "min_x2_plane_points_dist[1] : " << min_x2_plane_points_dist[1] << std::endl;

  if(x_plane1.nx * x_plane2.nx > 0) {
    // std::cout << "no xplane1 found " << std::endl;
    // std::cout << "no xplane2 found " << std::endl;
    found_x1_plane = false, found_x2_plane = false;
  } else {
    if(min_dist_x1 < room_dist_thres && min_x1_plane_points_dist[0] < plane_point_dist_thres && min_x1_plane_points_dist[1] < plane_point_dist_thres) {
      std::cout << "room has xplane1: " << x_plane1.nx << ", " << x_plane1.ny << ", " << x_plane1.nz << ", " << x_plane1.d << std::endl;
      found_x1_plane = true;
    } else {
      std::cout << "no xplane1 found " << std::endl;
      found_x1_plane = false;
    }
    if(min_dist_x2 < room_dist_thres && min_x2_plane_points_dist[0] < plane_point_dist_thres && min_x2_plane_points_dist[1] < plane_point_dist_thres) {
      std::cout << "room has xplane2: " << x_plane2.nx << ", " << x_plane2.ny << ", " << x_plane2.nz << ", " << x_plane2.d << std::endl;
      found_x2_plane = true;
    } else {
      std::cout << "no xplane2 found " << std::endl;
      found_x2_plane = false;
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

    std::cout << "y_plane1: " << y_plane.nx << ", " << y_plane.ny << ", " << y_plane.nz << ", " << y_plane.d << std::endl;
    float dist_y1 = -1 * (y_plane.nx * p_min.x + y_plane.ny * p_min.y);
    float diff_dist_y1 = 100;
    diff_dist_y1 = sqrt((dist_y1 - y_plane.d) * (dist_y1 - y_plane.d));

    // std::cout << "diff dist y1: " << diff_dist_y1 << std::endl;
    std::vector<float> diff_plane_points_dist = find_plane_points(top_right, bottom_right, y_plane);
    if(diff_plane_points_dist[0] < min_y1_plane_points_dist[0] && diff_plane_points_dist[1] < min_y1_plane_points_dist[1]) {
      min_dist_y1 = diff_dist_y1;
      min_y1_plane_points_dist[0] = diff_plane_points_dist[0];
      min_y1_plane_points_dist[1] = diff_plane_points_dist[1];
      y_plane1 = y_plane;
    }
  }

  for(const auto& y_plane : current_y_vert_planes) {
    if(y_plane.ny > 0) {
      continue;
    }

    std::cout << "y_plane2: " << y_plane.nx << ", " << y_plane.ny << ", " << y_plane.nz << ", " << y_plane.d << std::endl;
    float dist_y2 = -1 * (y_plane.nx * p_max.x + y_plane.ny * p_max.y);
    float diff_dist_y2 = 100;
    diff_dist_y2 = sqrt((dist_y2 - y_plane.d) * (dist_y2 - y_plane.d));

    // std::cout << "diff dist y2: " << diff_dist_y2 << std::endl;
    std::vector<float> diff_plane_points_dist = find_plane_points(bottom_left, top_left, y_plane);
    if(diff_plane_points_dist[0] < min_y2_plane_points_dist[0] && diff_plane_points_dist[1] < min_y2_plane_points_dist[1]) {
      min_dist_y2 = diff_dist_y2;
      min_y2_plane_points_dist[0] = diff_plane_points_dist[0];
      min_y2_plane_points_dist[1] = diff_plane_points_dist[1];
      y_plane2 = y_plane;
    }
  }

  std::cout << "selected yplane1 : " << y_plane1.nx << ", " << y_plane1.ny << ", " << y_plane1.nz << ", " << y_plane1.d << std::endl;
  std::cout << "min_dist_y1 : " << min_dist_y1 << std::endl;
  std::cout << "min_y1_plane_points_dist[0] : " << min_y1_plane_points_dist[0] << std::endl;
  std::cout << "min_y1_plane_points_dist[1] : " << min_y1_plane_points_dist[1] << std::endl;

  std::cout << "selected yplane2 : " << y_plane2.nx << ", " << y_plane2.ny << ", " << y_plane2.nz << ", " << y_plane2.d << std::endl;
  std::cout << "min_dist_y2 : " << min_dist_y2 << std::endl;
  std::cout << "min_y2_plane_points_dist[0] : " << min_y2_plane_points_dist[0] << std::endl;
  std::cout << "min_y2_plane_points_dist[1] : " << min_y2_plane_points_dist[1] << std::endl;

  if(y_plane1.ny * y_plane2.ny > 0) {
    // std::cout << "no yplane1 found " << std::endl;
    // std::cout << "no yplane2 found " << std::endl;
    found_y1_plane = false, found_y2_plane = false;
  } else {
    if(min_dist_y1 < room_dist_thres && min_y1_plane_points_dist[0] < plane_point_dist_thres && min_y1_plane_points_dist[1] < plane_point_dist_thres) {
      std::cout << "room has yplane1: " << y_plane1.nx << ", " << y_plane1.ny << ", " << y_plane1.nz << ", " << y_plane1.d << std::endl;
      found_y1_plane = true;
    } else {
      std::cout << "no yplane1 found " << std::endl;
      found_y1_plane = false;
    }
    if(min_dist_y2 < room_dist_thres && min_y2_plane_points_dist[0] < plane_point_dist_thres && min_y2_plane_points_dist[1] < plane_point_dist_thres) {
      std::cout << "room has yplane2: " << y_plane2.nx << ", " << y_plane2.ny << ", " << y_plane2.nz << ", " << y_plane2.d << std::endl;
      found_y2_plane = true;
    } else {
      std::cout << "no yplane2 found " << std::endl;
      found_y2_plane = false;
    }
  }
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
    if(min_dist_top_right < 1.0 && min_dist_bottom_left < 1.0) {
      top_right.x = cloud_hull->points[top_right_index].x;
      top_right.y = cloud_hull->points[top_right_index].y;

      bottom_right.x = cloud_hull->points[bottom_right_index].x;
      bottom_right.y = cloud_hull->points[bottom_right_index].y;

      top_left.x = cloud_hull->points[top_left_index].x;
      top_left.y = cloud_hull->points[top_left_index].y;

      bottom_left.x = cloud_hull->points[bottom_left_index].x;
      bottom_left.y = cloud_hull->points[bottom_left_index].y;

      return true;
    }
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

  std::cout << "closest_start_plane_point " << closest_start_plane_point.x << " ; " << closest_start_plane_point.y << std::endl;
  std::cout << "closest_end_plane_point " << closest_end_plane_point.x << " ; " << closest_end_plane_point.y << std::endl;
  plane_point_distances.push_back(min_start_point_plane_dist);
  plane_point_distances.push_back(min_end_point_plane_dist);

  return plane_point_distances;
}

}  // namespace s_graphs
