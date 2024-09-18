#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <pcl/io/pcd_io.h>

#include <boost/filesystem.hpp>
#include <s_graphs/common/graph_utils.hpp>
#include <s_graphs/common/keyframe.hpp>

namespace s_graphs {

KeyFrame::KeyFrame(const rclcpp::Time& stamp,
                   const Eigen::Isometry3d& odom,
                   double accum_distance,
                   const pcl::PointCloud<PointT>::Ptr& cloud,
                   const int floor_level,
                   const int session_id)
    : stamp(stamp),
      odom(odom),
      accum_distance(accum_distance),
      cloud(cloud),
      floor_level(floor_level),
      session_id(session_id),
      node(nullptr) {}

KeyFrame::KeyFrame(const std::string& directory,
                   const std::shared_ptr<GraphSLAM>& covisibility_graph)
    : stamp(),
      odom(Eigen::Isometry3d::Identity()),
      accum_distance(-1),
      cloud(nullptr),
      floor_level(0),
      session_id(0),
      node(nullptr) {
  load(directory, covisibility_graph);
}

KeyFrame::KeyFrame(const KeyFrame::Ptr& key) {
  stamp = key->stamp;
  odom = key->odom;
  accum_distance = key->accum_distance;
  cloud = key->cloud;
  dense_cloud = key->dense_cloud;
  x_plane_ids = key->x_plane_ids;
  y_plane_ids = key->y_plane_ids;
  hort_plane_ids = key->hort_plane_ids;
  floor_level = key->floor_level;

  node = new g2o::VertexSE3();
  node->setId(key->node->id());
  node->setEstimate(key->node->estimate());
}

void KeyFrame::set_dense_cloud(const pcl::PointCloud<PointT>::Ptr& cloud) {
  dense_cloud = cloud;
}

KeyFrame::~KeyFrame() {}

void KeyFrame::save(const std::string& directory, const int& sequential_id) {
  std::string kf_sub_directory = directory + "/" + std::to_string(sequential_id);

  if (!boost::filesystem::is_directory(kf_sub_directory)) {
    boost::filesystem::create_directory(kf_sub_directory);
  }

  std::ofstream ofs(kf_sub_directory + "/kf_data.txt");
  ofs << "stamp " << stamp.seconds() << " " << stamp.nanoseconds() << "\n";

  ofs << "estimate ";
  ofs << node->estimate().matrix() << "\n";

  ofs << "odom ";
  ofs << odom.matrix() << "\n";

  ofs << "accum_distance " << accum_distance << "\n";

  ofs << "floor_level " << floor_level << "\n";

  if (!x_plane_ids.empty()) {
    ofs << "x_plane_ids ";
    for (size_t i = 0; i < x_plane_ids.size(); i++) {
      if (i < x_plane_ids.size() - 1)
        ofs << x_plane_ids[i] << " ";
      else
        ofs << x_plane_ids[i];
    }
    ofs << "\n";
  }

  if (!y_plane_ids.empty()) {
    ofs << "y_plane_ids ";
    for (size_t i = 0; i < y_plane_ids.size(); i++) {
      if (i < y_plane_ids.size() - 1)
        ofs << y_plane_ids[i] << " ";
      else
        ofs << y_plane_ids[i];
    }
    ofs << "\n";
  }

  if (!hort_plane_ids.empty()) {
    ofs << "hort_plane_ids ";
    for (size_t i = 0; i < hort_plane_ids.size(); i++) {
      if (i < hort_plane_ids.size() - 1)
        ofs << hort_plane_ids[i] << " ";
      else
        ofs << hort_plane_ids[i];
    }
    ofs << "\n";
  }

  if (floor_coeffs) {
    ofs << "floor_coeffs " << floor_coeffs->transpose() << "\n";
  }

  if (utm_coord) {
    ofs << "utm_coord " << utm_coord->transpose() << "\n";
  }

  if (acceleration) {
    ofs << "acceleration " << acceleration->transpose() << "\n";
  }

  if (orientation) {
    ofs << "orientation " << orientation->w() << " " << orientation->x() << " "
        << orientation->y() << " " << orientation->z() << "\n";
  }

  if (node) {
    ofs << "id " << node->id() << "\n";
  }

  bool marginalized_kf = GraphUtils::get_keyframe_marg_data(node);
  if (marginalized_kf) {
    ofs << "marginalized " << marginalized_kf << "\n";
  }

  bool stair_kf = GraphUtils::get_keyframe_stair_data(node);
  if (stair_kf) {
    ofs << "stair_kf " << stair_kf << "\n";
  }

  ofs.close();

  if (cloud) pcl::io::savePCDFileBinary(kf_sub_directory + "/cloud.pcd", *cloud);
  if (dense_cloud)
    pcl::io::savePCDFileBinary(kf_sub_directory + "/dense_cloud.pcd", *dense_cloud);
}

bool KeyFrame::load(const std::string& directory,
                    const std::shared_ptr<GraphSLAM>& covisibility_graph) {
  std::ifstream ifs(directory + "/kf_data.txt");
  if (!ifs) {
    return false;
  }

  int node_id = -1;
  bool marginalized_kf = false;
  bool stair_kf = false;
  boost::optional<Eigen::Isometry3d> estimate;
  while (!ifs.eof()) {
    std::string token;
    ifs >> token;

    if (token == "stamp") {
      double seconds;
      double nanoseconds;
      ifs >> seconds >> nanoseconds;
      stamp = rclcpp::Time(seconds, nanoseconds);
    } else if (token == "estimate") {
      Eigen::Matrix4d mat;
      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
          ifs >> mat(i, j);
        }
      }
      estimate = Eigen::Isometry3d::Identity();
      estimate->linear() = mat.block<3, 3>(0, 0);
      estimate->translation() = mat.block<3, 1>(0, 3);
      odom.linear() = mat.block<3, 3>(0, 0);
      odom.translation() = mat.block<3, 1>(0, 3);

    } else if (token == "odom") {
      Eigen::Matrix4d odom_mat = Eigen::Matrix4d::Identity();
      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
          ifs >> odom_mat(i, j);
        }
      }
      odom.setIdentity();
      odom.linear() = odom_mat.block<3, 3>(0, 0);
      odom.translation() = odom_mat.block<3, 1>(0, 3);
    } else if (token == "floor_level") {
      ifs >> floor_level;
    } else if (token == "accum_distance") {
      double distance;
      ifs >> accum_distance;
    } else if (token == "x_plane_ids") {
      int x_plane_id;
      while (ifs >> x_plane_id) {
        x_plane_ids.push_back(x_plane_id);
        if (ifs.peek() == '\n' || ifs.peek() == '\r') break;
      }
    } else if (token == "y_plane_ids") {
      int y_plane_id;
      while (ifs >> y_plane_id) {
        y_plane_ids.push_back(y_plane_id);
        if (ifs.peek() == '\n' || ifs.peek() == '\r') break;
      }
    } else if (token == "hort_plane_ids") {
      int hort_plane_id;
      while (ifs >> hort_plane_id) {
        hort_plane_ids.push_back(hort_plane_id);
        if (ifs.peek() == '\n' || ifs.peek() == '\r') break;
      }
    } else if (token == "floor_coeffs") {
      Eigen::Vector4d coeffs;
      ifs >> coeffs[0] >> coeffs[1] >> coeffs[2] >> coeffs[3];
      floor_coeffs = coeffs;
    } else if (token == "utm_coord") {
      Eigen::Vector3d coord;
      ifs >> coord[0] >> coord[1] >> coord[2];
      utm_coord = coord;
    } else if (token == "acceleration") {
      Eigen::Vector3d acc;
      ifs >> acc[0] >> acc[1] >> acc[2];
      acceleration = acc;
    } else if (token == "orientation") {
      Eigen::Quaterniond quat;
      ifs >> quat.w() >> quat.x() >> quat.y() >> quat.z();
      orientation = quat;
    } else if (token == "id") {
      int id;
      ifs >> id;
      node_id = id;
    } else if (token == "marginalized") {
      ifs >> marginalized_kf;
    } else if (token == "stair_kf") {
      ifs >> stair_kf;
    }
  }

  if (node_id < 0) {
    std::cerr << "invalid node id!!" << std::endl;
    std::cerr << directory << std::endl;
    return false;
  }

  g2o::VertexSE3* kf_node(new g2o::VertexSE3());
  kf_node->setId(node_id);
  kf_node->setEstimate(*estimate);
  node = covisibility_graph->copy_se3_node(kf_node);

  if (marginalized_kf) {
    GraphUtils::set_keyframe_marg_data(node, marginalized_kf);
  }

  if (stair_kf) {
    GraphUtils::set_keyframe_stair_data(node, stair_kf);
  }

  pcl::PointCloud<PointT>::Ptr kf_cloud(new pcl::PointCloud<PointT>());
  pcl::io::loadPCDFile(directory + "/cloud.pcd", *kf_cloud);
  cloud = kf_cloud;

  pcl::PointCloud<PointT>::Ptr kf_dense_cloud(new pcl::PointCloud<PointT>());
  pcl::io::loadPCDFile(directory + "/dense_cloud.pcd", *kf_dense_cloud);
  dense_cloud = kf_dense_cloud;

  return true;
}

long KeyFrame::id() const { return node->id(); }

Eigen::Isometry3d KeyFrame::estimate() const { return node->estimate(); }

KeyFrameSnapshot::KeyFrameSnapshot(const Eigen::Isometry3d& pose,
                                   const pcl::PointCloud<PointT>::ConstPtr& cloud,
                                   const bool marginalized,
                                   const int floor_level)
    : pose(pose),
      cloud(cloud),
      k_marginalized(marginalized),
      floor_level(floor_level) {}

KeyFrameSnapshot::KeyFrameSnapshot(const KeyFrame::Ptr& key) {
  pose = key->node->estimate();
  cloud = key->cloud;
  k_marginalized = GraphUtils::get_keyframe_marg_data(key->node);
  floor_level = key->floor_level;
}

KeyFrameSnapshot::~KeyFrameSnapshot() {}

}  // namespace s_graphs
