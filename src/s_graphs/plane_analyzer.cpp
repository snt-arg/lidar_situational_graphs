// SPDX-License-Identifier: BSD-2-Clause

#include <s_graphs/plane_analyzer.hpp>

namespace s_graphs {

PlaneAnalyzer::PlaneAnalyzer(ros::NodeHandle private_nh) {
  min_seg_points_ = private_nh.param<int>("min_seg_points", 100);
  min_horizontal_inliers = private_nh.param<int>("min_horizontal_inliers", 500);
  min_vertical_inliers = private_nh.param<int>("min_vertical_inliers", 100);
  use_euclidean_filter = private_nh.param<bool>("use_euclidean_filter", true);
  use_shadow_filter = private_nh.param<bool>("use_shadow_filter", false);
  plane_extraction_frame = private_nh.param<std::string>("plane_extraction_frame_id", "base_link");
  plane_visualization_frame = private_nh.param<std::string>("plane_visualization_frame_id", "base_link_elevated");

  init_ros(private_nh);
}

PlaneAnalyzer::~PlaneAnalyzer() {}

void PlaneAnalyzer::init_ros(ros::NodeHandle private_nh) {
  segmented_cloud_pub = private_nh.advertise<sensor_msgs::PointCloud2>("segmented_cloud", 1);
}

std::vector<pcl::PointCloud<PointNormal>::Ptr> PlaneAnalyzer::extract_segmented_planes(const pcl::PointCloud<PointT>::ConstPtr cloud) {
  pcl::PointCloud<PointNormal>::Ptr segmented_cloud(new pcl::PointCloud<PointNormal>);
  std::vector<pcl::PointCloud<PointNormal>::Ptr> extracted_cloud_vec;
  pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);

  transformed_cloud->header = cloud->header;
  for(size_t i = 0; i < cloud->points.size(); ++i) {
    transformed_cloud->points.push_back(cloud->points[i]);
  }

  int i = 0;
  while(transformed_cloud->points.size() > min_seg_points_) {
    try {
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      pcl::ExtractIndices<PointT> extract;
      // Optional
      pcl::SACSegmentation<PointT> seg;
      seg.setOptimizeCoefficients(true);
      // Mandatory
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(0.01);
      seg.setInputCloud(transformed_cloud);
      seg.setNumberOfThreads(8);
      // seg.setInputNormals(normal_cloud);
      // int model_type = seg.getModelType();
      seg.segment(*inliers, *coefficients);
      /* check if indicies are not empty for no crash */
      if(inliers->indices.empty()) {
        std::cout << "Breaking as no model found" << std::endl;
        break;
      }
      /* filtering out noisy ground plane measurements */
      if((fabs(coefficients->values[2]) > 0.9 && inliers->indices.size() < min_horizontal_inliers) || inliers->indices.size() < min_vertical_inliers) {
        extract.setInputCloud(transformed_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*transformed_cloud);
        continue;
      }
      // std::cout << "Model coefficients before " << std::to_string(i) << ": " << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << " " << coefficients->values[3] << std::endl;

      Eigen::Vector4d normal(coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
      Eigen::Vector3d closest_point = normal.head(3) * normal(3);
      Eigen::Vector4d plane;
      plane.head(3) = closest_point / closest_point.norm();
      plane(3) = closest_point.norm();

      // Eigen::Vector4f normals_flipped = normal.cast<float>();
      // pcl::flipNormalTowardsViewpoint(transformed_cloud->points[inliers->indices[0]], 0, 0, 0, normals_flipped);
      // std::cout << "Model coefficients after " << std::to_string(i) << ": " << normals_flipped << std::endl;

      pcl::PointCloud<PointNormal>::Ptr extracted_cloud(new pcl::PointCloud<PointNormal>);
      for(const auto& idx : inliers->indices) {
        PointNormal tmp_cloud;
        tmp_cloud.x = transformed_cloud->points[idx].x;
        tmp_cloud.y = transformed_cloud->points[idx].y;
        tmp_cloud.z = transformed_cloud->points[idx].z;
        tmp_cloud.normal_x = plane(0);
        tmp_cloud.normal_y = plane(1);
        tmp_cloud.normal_z = plane(2);
        tmp_cloud.curvature = plane(3);

        extracted_cloud->points.push_back(tmp_cloud);
      }

      pcl::PointCloud<PointNormal>::Ptr extracted_cloud_filtered;
      if(use_euclidean_filter)
        extracted_cloud_filtered = compute_clusters(extracted_cloud);
      else if(use_shadow_filter) {
        pcl::PointCloud<pcl::Normal>::Ptr normals = compute_cloud_normals(extracted_cloud);
        extracted_cloud_filtered = shadow_filter(extracted_cloud, normals);
      } else {
        extracted_cloud_filtered = extracted_cloud;
      }

      // visulazing the pointcloud
      for(int pc = 0; pc < extracted_cloud_filtered->points.size(); ++pc) {
        segmented_cloud->points.push_back(extracted_cloud_filtered->points[pc]);
        // segmented_cloud->back().r = 254;
        // segmented_cloud->back().g = 216;
        // segmented_cloud->back().b = 177;
      }

      extracted_cloud_vec.push_back(extracted_cloud_filtered);

      // sensor_msgs::PointCloud2 extracted_cloud_msg;
      // pcl::toROSMsg(*extracted_cloud_filtered, extracted_cloud_msg);
      // std_msgs::Header ext_msg_header = pcl_conversions::fromPCL(transformed_cloud->header);
      // extracted_cloud_msg.header = ext_msg_header;
      // extracted_cloud_msg.header.frame_id = plane_extraction_frame;
      // extracted_cloud_vec.push_back(extracted_cloud_msg);

      extract.setInputCloud(transformed_cloud);
      extract.setIndices(inliers);
      extract.setNegative(true);
      extract.filter(*transformed_cloud);
      i++;
    } catch(const std::exception& e) {
      std::cout << "No ransac model found" << std::endl;
      break;
    }
  }

  sensor_msgs::PointCloud2 segmented_cloud_msg;
  pcl::toROSMsg(*segmented_cloud, segmented_cloud_msg);
  std_msgs::Header msg_header = pcl_conversions::fromPCL(transformed_cloud->header);
  segmented_cloud_msg.header = msg_header;
  segmented_cloud_msg.header.frame_id = plane_visualization_frame;
  segmented_cloud_pub.publish(segmented_cloud_msg);

  return extracted_cloud_vec;
}

pcl::PointCloud<PointNormal>::Ptr PlaneAnalyzer::compute_clusters(const pcl::PointCloud<PointNormal>::Ptr& extracted_cloud) {
  pcl::search::KdTree<PointNormal>::Ptr tree(new pcl::search::KdTree<PointNormal>);
  tree->setInputCloud(extracted_cloud);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointNormal> ec;
  ec.setClusterTolerance(0.5);
  ec.setMinClusterSize(10);
  ec.setMaxClusterSize(250000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(extracted_cloud);
  ec.extract(cluster_indices);

  // auto max_iterator = std::max_element(std::begin(cluster_indices), std::end(cluster_indices), [](const pcl::PointIndices& lhs, const pcl::PointIndices& rhs) { return lhs.indices.size() < rhs.indices.size(); });

  pcl::PointCloud<PointNormal>::Ptr cloud_cluster(new pcl::PointCloud<PointNormal>);
  int cluster_id = 0;
  for(auto single_cluster : cluster_indices) {
    // std_msgs::ColorRGBA color = rainbow_color_map((single_cluster).indices.size() % 100 / 10.0);
    std_msgs::ColorRGBA color = random_color();
    double r = color.r;
    double g = color.g;
    double b = color.b;
    for(const auto& idx : (single_cluster).indices) {
      cloud_cluster->push_back(extracted_cloud->points[idx]);
      cloud_cluster->width = cloud_cluster->size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      cloud_cluster->back().normal_x = extracted_cloud->back().normal_x;
      cloud_cluster->back().normal_y = extracted_cloud->back().normal_y;
      cloud_cluster->back().normal_z = extracted_cloud->back().normal_z;
      cloud_cluster->back().curvature = extracted_cloud->back().curvature;
      cloud_cluster->back().r = r;
      cloud_cluster->back().g = g;
      cloud_cluster->back().b = b;
    }
    cluster_id++;
  }

  return cloud_cluster;
}

pcl::PointCloud<pcl::Normal>::Ptr PlaneAnalyzer::compute_cloud_normals(const pcl::PointCloud<PointNormal>::Ptr& extracted_cloud) {
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<PointNormal, pcl::Normal> ne;
  pcl::search::KdTree<PointNormal>::Ptr tree(new pcl::search::KdTree<PointNormal>());

  ne.setInputCloud(extracted_cloud);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.1);
  ne.compute(*cloud_normals);

  return cloud_normals;
}

pcl::PointCloud<PointNormal>::Ptr PlaneAnalyzer::shadow_filter(const pcl::PointCloud<PointNormal>::Ptr& extracted_cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals) {
  pcl::PointCloud<PointNormal>::Ptr extracted_cloud_filtered(new pcl::PointCloud<PointNormal>);
  pcl::ShadowPoints<PointNormal, pcl::Normal> sp_filter;
  sp_filter.setNormals(cloud_normals);
  sp_filter.setThreshold(0.1);
  sp_filter.setInputCloud(extracted_cloud);
  sp_filter.filter(*extracted_cloud_filtered);

  return extracted_cloud_filtered;
}

std_msgs::ColorRGBA PlaneAnalyzer::rainbow_color_map(double h) {
  std_msgs::ColorRGBA color;
  color.a = 255;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if(!(i & 1)) f = 1 - f;  // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch(i) {
    case 6:
    case 0:
      color.r = 255 * v;
      color.g = 255 * n;
      color.b = 255 * m;
      break;
    case 1:
      color.r = 255 * n;
      color.g = 255 * v;
      color.b = 255 * m;
      break;
    case 2:
      color.r = 255 * m;
      color.g = 255 * v;
      color.b = 255 * n;
      break;
    case 3:
      color.r = 255 * m;
      color.g = 255 * n;
      color.b = 255 * v;
      break;
    case 4:
      color.r = 255 * n;
      color.g = 255 * m;
      color.b = 255 * v;
      break;
    case 5:
      color.r = 255 * v;
      color.g = 255 * m;
      color.b = 255 * n;
      break;
    default:
      color.r = 255;
      color.g = 127;
      color.b = 127;
      break;
  }
  return color;
}

std_msgs::ColorRGBA PlaneAnalyzer::random_color() {
  std_msgs::ColorRGBA color;
  color.r = rand() % 256;
  color.b = rand() % 256;
  color.g = rand() % 256;

  return color;
}

}  // namespace s_graphs
