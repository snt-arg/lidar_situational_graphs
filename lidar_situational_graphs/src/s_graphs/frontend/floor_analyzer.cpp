#include <s_graphs/frontend/floor_analyzer.hpp>

namespace s_graphs {

FloorAnalyzer::FloorAnalyzer() {}

FloorAnalyzer::~FloorAnalyzer() {}

void FloorAnalyzer::perform_floor_segmentation(
    const std::vector<situational_graphs_msgs::msg::PlaneData>& current_x_vert_planes,
    const std::vector<situational_graphs_msgs::msg::PlaneData>& current_y_vert_planes,
    std::vector<situational_graphs_msgs::msg::PlaneData>& floor_plane_candidates_vec) {
  // analyze the largest x plane pair
  situational_graphs_msgs::msg::PlaneData floor_x_plane1, floor_x_plane2;
  situational_graphs_msgs::msg::PlaneData floor_y_plane1, floor_y_plane2;
  floor_x_plane1.nx = -1;
  floor_x_plane2.nx = -1;
  floor_y_plane1.nx = -1;
  floor_y_plane2.nx = -1;

  float max_xplane_width = 0;
  for (auto x_plane1 : current_x_vert_planes) {
    if (x_plane1.nx < 0) {
      continue;
    }
    for (auto x_plane2 : current_x_vert_planes) {
      if (x_plane2.nx > 0) {
        continue;
      }

      float x_plane_width = PlaneUtils::width_between_planes(x_plane1, x_plane2);
      bool planes_placed_correctly = false;
      if (!x_plane1.plane_points.empty() && !x_plane2.plane_points.empty()) {
        planes_placed_correctly = PlaneUtils::compute_point_difference(
            x_plane1.plane_points.back().x, x_plane2.plane_points.back().x);
      } else
        continue;

      float dot_product = PlaneUtils::plane_dot_product(x_plane1, x_plane2);
      if (abs(dot_product) < 0.9) continue;

      if (x_plane_width > max_xplane_width && planes_placed_correctly) {
        max_xplane_width = x_plane_width;
        floor_x_plane1 = x_plane1;
        floor_x_plane2 = x_plane2;
      }
    }
  }

  float max_yplane_width = 0;
  for (auto y_plane1 : current_y_vert_planes) {
    if (y_plane1.ny < 0) {
      continue;
    }
    for (auto y_plane2 : current_y_vert_planes) {
      if (y_plane2.ny > 0) {
        continue;
      }

      float y_plane_width = PlaneUtils::width_between_planes(y_plane1, y_plane2);
      bool planes_placed_correctly = false;
      if (!y_plane1.plane_points.empty() && !y_plane2.plane_points.empty()) {
        planes_placed_correctly = PlaneUtils::compute_point_difference(
            y_plane1.plane_points.back().y, y_plane2.plane_points.back().y);
      } else
        continue;

      float dot_product = PlaneUtils::plane_dot_product(y_plane1, y_plane2);
      if (abs(dot_product) < 0.9) continue;

      if (y_plane_width > max_yplane_width && planes_placed_correctly) {
        max_yplane_width = y_plane_width;
        floor_y_plane1 = y_plane1;
        floor_y_plane2 = y_plane2;
      }
    }
  }

  // std::cout << "xplane1: " << floor_x_plane1.nx << ", " << floor_x_plane1.ny << ", "
  // << floor_x_plane1.nz << ", " << floor_x_plane1.d << std::endl; std::cout <<
  // "xplane2: " << floor_x_plane2.nx << ", " << floor_x_plane2.ny << ", " <<
  // floor_x_plane2.nz << ", " << floor_x_plane2.d << std::endl; std::cout << "yplane1:
  // " << floor_y_plane1.nx << ", " << floor_y_plane1.ny << ", " << floor_y_plane1.nz <<
  // ", " << floor_y_plane1.d << std::endl; std::cout << "yplane2: " <<
  // floor_y_plane2.nx << ", " << floor_y_plane2.ny << ", " << floor_y_plane2.nz << ", "
  // << floor_y_plane2.d << std::endl;

  if (floor_x_plane1.nx != -1) floor_plane_candidates_vec.push_back(floor_x_plane1);
  if (floor_x_plane2.nx != -1) floor_plane_candidates_vec.push_back(floor_x_plane2);
  if (floor_y_plane1.nx != -1) floor_plane_candidates_vec.push_back(floor_y_plane1);
  if (floor_y_plane2.nx != -1) floor_plane_candidates_vec.push_back(floor_y_plane2);
}

}  // namespace s_graphs
