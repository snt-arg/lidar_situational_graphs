// SPDX-License-Identifier: BSD-2-Clause

#include <s_graphs/neighbour_mapper.hpp>

namespace s_graphs {

NeighbourMapper::NeighbourMapper(const ros::NodeHandle& private_nh) {}

NeighbourMapper::~NeighbourMapper() {}

void NeighbourMapper::detect_room_neighbours(std::unique_ptr<GraphSLAM>& graph_slam, const s_graphs::RoomsData& room_msg, std::vector<Corridors>& x_corridors, std::vector<Corridors>& y_corridors, std::vector<Rooms>& rooms_vec) {
  float min_dist_room_room = 100;
  float min_dist_room_x_corridor = 100;
  float min_dist_room_y_corridor = 100;
  float matching_threshold = 0.5;

  s_graphs::Rooms* mapped_room;
  s_graphs::Corridors* mapped_x_corridor;
  s_graphs::Corridors* mapped_y_corridor;
  for(const auto& room_data : room_msg.rooms) {
    for(auto& current_room : rooms_vec) {
      float dist_room_room = sqrt(pow(room_data.room_center.x - current_room.node->estimate()(0), 2) + pow(room_data.room_center.y - current_room.node->estimate()(1), 2));
      if(dist_room_room < min_dist_room_room) {
        min_dist_room_room = dist_room_room;
        mapped_room = &current_room;
      }
    }

    for(auto& current_x_corridor : x_corridors) {
      float dist_room_x_corridor = sqrt(pow(room_data.room_center.x - current_x_corridor.node->estimate(), 2) + pow(room_data.room_center.y - current_x_corridor.keyframe_trans(1), 2));
      if(dist_room_x_corridor < min_dist_room_x_corridor) {
        min_dist_room_x_corridor = dist_room_x_corridor;
        mapped_x_corridor = &current_x_corridor;
      }
    }

    for(auto& current_y_corridor : y_corridors) {
      float dist_room_y_corridor = sqrt(pow(room_data.room_center.x - current_y_corridor.keyframe_trans(0), 2) + pow(room_data.room_center.y - current_y_corridor.node->estimate(), 2));
      if(dist_room_y_corridor < min_dist_room_y_corridor) {
        min_dist_room_y_corridor = dist_room_y_corridor;
        mapped_y_corridor = &current_y_corridor;
      }
    }

    if(min_dist_room_room < min_dist_room_x_corridor && min_dist_room_room < min_dist_room_y_corridor && min_dist_room_room < matching_threshold) {
      mapped_room->connected_id = room_data.id;
      mapped_room->connected_neighbour_ids = room_data.neighbour_ids;
    } else if(min_dist_room_x_corridor < min_dist_room_room && min_dist_room_x_corridor < min_dist_room_y_corridor && min_dist_room_x_corridor < matching_threshold) {
      mapped_x_corridor->connected_id = room_data.id;
      mapped_x_corridor->connected_neighbour_ids = room_data.neighbour_ids;
    } else if(min_dist_room_y_corridor < min_dist_room_room && min_dist_room_y_corridor < min_dist_room_x_corridor && min_dist_room_y_corridor < matching_threshold) {
      mapped_y_corridor->connected_id = room_data.id;
      mapped_y_corridor->connected_neighbour_ids = room_data.neighbour_ids;
    }
  }
}

void NeighbourMapper::factor_room_neighbours(std::unique_ptr<GraphSLAM>& graph_slam, const s_graphs::RoomsData& room_msg, std::vector<Corridors>& x_corridors, std::vector<Corridors>& y_corridors, std::vector<Rooms>& rooms_vec) {
  // here we have all the room detected connected ids filled it along with its neighbours
  for(const auto& room_data : room_msg.rooms) {
    // get the room node in the graph with detected room connected id
    auto found_current_room = std::find_if(rooms_vec.begin(), rooms_vec.end(), boost::bind(&Rooms::connected_id, _1) == room_data.id);
    if(found_current_room != rooms_vec.end()) {
      // loop over the connected_ids of the found room
      for(const auto& connected_neighbour_id : (*found_current_room).connected_neighbour_ids) {
        auto found_detected_neighbour = std::find_if(room_msg.rooms.begin(), room_msg.rooms.end(), boost::bind(&s_graphs::RoomData::id, _1) == connected_neighbour_id);
        // find each connected neighbours node in the graph
        auto found_current_room_neighbour = std::find_if(rooms_vec.begin(), rooms_vec.end(), boost::bind(&Rooms::connected_id, _1) == connected_neighbour_id);
        // if the found neighbour is a room, add room/room edge
        if(found_current_room_neighbour != rooms_vec.end()) {
          Eigen::Vector2d room_room_meas = room_room_measurement(room_data, (*found_detected_neighbour));
          factor_room_room_constraints(graph_slam, (*found_current_room), (*found_current_room_neighbour), room_room_meas);
        } else {
          auto found_current_x_corridor_neighbour = std::find_if(x_corridors.begin(), x_corridors.end(), boost::bind(&Corridors::connected_id, _1) == connected_neighbour_id);
          // if the found neighbour is x_corridor add room/x_corridor edge
          if(found_current_x_corridor_neighbour != x_corridors.end()) {
            double room_x_corr_meas = room_x_corridor_measurement(room_data, (*found_detected_neighbour));
            factor_room_x_corridor_constraints(graph_slam, (*found_current_room), (*found_current_x_corridor_neighbour), room_x_corr_meas);
          } else {
            auto found_current_y_corridor_neighbour = std::find_if(y_corridors.begin(), y_corridors.end(), boost::bind(&Corridors::connected_id, _1) == connected_neighbour_id);
            // if the found neighbour is y_corridor add room/y_corridor edge
            if(found_current_y_corridor_neighbour != y_corridors.end()) {
              double room_y_corr_meas = room_y_corridor_measurement(room_data, (*found_detected_neighbour));
              factor_room_y_corridor_constraints(graph_slam, (*found_current_room), (*found_current_y_corridor_neighbour), room_y_corr_meas);
            }
          }
        }
      }
      (*found_current_room).connected_id = -1;
      (*found_current_room).connected_neighbour_ids.clear();
    } else {
      auto found_current_x_corridor = std::find_if(x_corridors.begin(), x_corridors.end(), boost::bind(&Corridors::connected_id, _1) == room_data.id);
      if(found_current_x_corridor != x_corridors.end()) {
        // loop over the current x_corridor neighbours
        for(const auto& connected_neighbour_id : (*found_current_x_corridor).connected_neighbour_ids) {
          auto found_detected_neighbour = std::find_if(room_msg.rooms.begin(), room_msg.rooms.end(), boost::bind(&s_graphs::RoomData::id, _1) == connected_neighbour_id);
          // find each connected neighbours node in the graph
          auto found_current_room_neighbour = std::find_if(rooms_vec.begin(), rooms_vec.end(), boost::bind(&Rooms::connected_id, _1) == connected_neighbour_id);
          if(found_current_room_neighbour != rooms_vec.end()) {
            double x_corr_room_meas = room_x_corridor_measurement((*found_detected_neighbour), room_data);
            factor_x_corridor_room_constraints(graph_slam, (*found_current_x_corridor), (*found_current_room_neighbour), x_corr_room_meas);
          } else {
            auto found_current_x_corridor_neighbour = std::find_if(x_corridors.begin(), x_corridors.end(), boost::bind(&Corridors::connected_id, _1) == connected_neighbour_id);
            if(found_current_x_corridor_neighbour != x_corridors.end()) {
              double x_corr_x_corr_meas = x_corridor_x_corridor_measurement(room_data, (*found_detected_neighbour));
              factor_x_corridor_x_corridor_constraints(graph_slam, (*found_current_x_corridor), (*found_current_x_corridor_neighbour), x_corr_x_corr_meas);
            } else {
              auto found_current_y_corridor_neighbour = std::find_if(y_corridors.begin(), y_corridors.end(), boost::bind(&Corridors::connected_id, _1) == connected_neighbour_id);
              if(found_current_y_corridor_neighbour != y_corridors.end()) {
                // not adding edge here
                factor_x_corridor_y_corridor_constraints(graph_slam, (*found_current_x_corridor), (*found_current_y_corridor_neighbour));
              }
            }
          }
        }
        (*found_current_x_corridor).connected_id = -1;
        (*found_current_x_corridor).connected_neighbour_ids.clear();
      } else {
        auto found_current_y_corridor = std::find_if(y_corridors.begin(), y_corridors.end(), boost::bind(&Corridors::connected_id, _1) == room_data.id);
        if(found_current_y_corridor != y_corridors.end()) {
          // loop over the current y_corridor neighbours
          for(const auto& connected_neighbour_id : (*found_current_y_corridor).connected_neighbour_ids) {
            auto found_detected_neighbour = std::find_if(room_msg.rooms.begin(), room_msg.rooms.end(), boost::bind(&s_graphs::RoomData::id, _1) == connected_neighbour_id);
            // find each connected neighbours node in the graph
            auto found_current_room_neighbour = std::find_if(rooms_vec.begin(), rooms_vec.end(), boost::bind(&Rooms::connected_id, _1) == connected_neighbour_id);
            if(found_current_room_neighbour != rooms_vec.end()) {
              double y_corr_room_meas = room_y_corridor_measurement(room_data, (*found_detected_neighbour));
              factor_y_corridor_room_constraints(graph_slam, (*found_current_y_corridor), (*found_current_room_neighbour), y_corr_room_meas);
            } else {
              auto found_current_x_corridor_neighbour = std::find_if(x_corridors.begin(), x_corridors.end(), boost::bind(&Corridors::connected_id, _1) == connected_neighbour_id);
              if(found_current_x_corridor_neighbour != x_corridors.end()) {
                // not adding any edge here
                factor_y_corridor_x_corridor_constraints(graph_slam, (*found_current_y_corridor), (*found_current_x_corridor_neighbour));
              } else {
                auto found_current_y_corridor_neighbour = std::find_if(y_corridors.begin(), y_corridors.end(), boost::bind(&Corridors::connected_id, _1) == connected_neighbour_id);
                if(found_current_y_corridor_neighbour != y_corridors.end()) {
                  double y_corr_y_corr_meas = y_corridor_y_corridor_measurement(room_data, (*found_detected_neighbour));
                  factor_y_corridor_y_corridor_constraints(graph_slam, (*found_current_y_corridor), (*found_current_y_corridor_neighbour), y_corr_y_corr_meas);
                }
              }
            }
          }
          (*found_current_y_corridor).connected_id = -1;
          (*found_current_y_corridor).connected_neighbour_ids.clear();
        }
      }
    }
  }
}

Eigen::Vector2d NeighbourMapper::room_room_measurement(const s_graphs::RoomData& room_msg_1, const s_graphs::RoomData& room_msg_2) {
  Eigen::Vector2d room_room_meas(0, 0);
  room_room_meas(0) = pow(room_msg_1.room_center.x - room_msg_2.room_center.x, 2);
  room_room_meas(1) = pow(room_msg_1.room_center.y - room_msg_2.room_center.y, 2);

  return room_room_meas;
}

double NeighbourMapper::room_x_corridor_measurement(const s_graphs::RoomData& room_msg, const s_graphs::RoomData& x_corridor_msg) {
  double room_x_corr_meas;
  room_x_corr_meas = pow(room_msg.room_center.x - x_corridor_msg.room_center.x, 2);

  return room_x_corr_meas;
}

double NeighbourMapper::room_y_corridor_measurement(const s_graphs::RoomData& room_msg, const s_graphs::RoomData& y_corridor_msg) {
  double room_y_corr_meas;
  room_y_corr_meas = pow(room_msg.room_center.y - y_corridor_msg.room_center.y, 2);

  return room_y_corr_meas;
}

double NeighbourMapper::x_corridor_x_corridor_measurement(const s_graphs::RoomData& x_corridor_msg1, const s_graphs::RoomData& x_corridor_msg2) {
  double x_corr_x_corr_meas;
  x_corr_x_corr_meas = pow(x_corridor_msg1.room_center.x - x_corridor_msg2.room_center.x, 2);

  return x_corr_x_corr_meas;
}

double NeighbourMapper::y_corridor_y_corridor_measurement(const s_graphs::RoomData& y_corridor_msg1, const s_graphs::RoomData& y_corridor_msg2) {
  double y_corr_y_corr_meas;
  y_corr_y_corr_meas = pow(y_corridor_msg1.room_center.y - y_corridor_msg2.room_center.y, 2);

  return y_corr_y_corr_meas;
}

void NeighbourMapper::factor_room_room_constraints(std::unique_ptr<GraphSLAM>& graph_slam, s_graphs::Rooms& room1, const s_graphs::Rooms& room2, Eigen::Vector2d room_room_meas) {
  bool neighbour_exists = false;
  for(const auto& room1_neighbour_id : room1.neighbour_ids) {
    if(room2.id == room1_neighbour_id) {
      neighbour_exists = true;
      break;
    }
  }
  if(!neighbour_exists) room1.neighbour_ids.push_back(room2.id);

  // TODO:HB add an edge in the graph between room1 and room2
  Eigen::Matrix2d information;
  information.setIdentity();
  auto edge = graph_slam->add_room_room_edge(room1.node, room2.node, room_room_meas, information);
  graph_slam->add_robust_kernel(edge, "Huber", 1.0);
}

void NeighbourMapper::factor_room_x_corridor_constraints(std::unique_ptr<GraphSLAM>& graph_slam, s_graphs::Rooms& room, const s_graphs::Corridors& x_corridor, double room_x_corr_meas) {
  bool neighbour_exists = false;
  for(const auto& room_neighbour_id : room.neighbour_ids) {
    if(x_corridor.id == room_neighbour_id) {
      neighbour_exists = true;
      break;
    }
  }
  if(!neighbour_exists) room.neighbour_ids.push_back(x_corridor.id);

  // TODO:HB add an edge in the graph between room1 and x_corridor
  Eigen::Matrix<double, 1, 1> information(1);
  auto edge = graph_slam->add_room_x_corridor_edge(room.node, x_corridor.node, room_x_corr_meas, information);
  graph_slam->add_robust_kernel(edge, "Huber", 1.0);
}

void NeighbourMapper::factor_room_y_corridor_constraints(std::unique_ptr<GraphSLAM>& graph_slam, s_graphs::Rooms& room, const s_graphs::Corridors& y_corridor, double room_y_corr_meas) {
  bool neighbour_exists = false;
  for(const auto& room_neighbour_id : room.neighbour_ids) {
    if(y_corridor.id == room_neighbour_id) {
      neighbour_exists = true;
      break;
    }
  }
  if(!neighbour_exists) room.neighbour_ids.push_back(y_corridor.id);

  Eigen::Matrix<double, 1, 1> information(1);
  auto edge = graph_slam->add_room_y_corridor_edge(room.node, y_corridor.node, room_y_corr_meas, information);
  graph_slam->add_robust_kernel(edge, "Huber", 1.0);
}

void NeighbourMapper::factor_x_corridor_room_constraints(std::unique_ptr<GraphSLAM>& graph_slam, s_graphs::Corridors& x_corridor, const s_graphs::Rooms& room, double x_corr_room_meas) {
  bool neighbour_exists = false;
  for(const auto& x_corridor_neighbour_id : x_corridor.neighbour_ids) {
    if(room.id == x_corridor_neighbour_id) {
      neighbour_exists = true;
      break;
    }
  }
  if(!neighbour_exists) x_corridor.neighbour_ids.push_back(room.id);

  Eigen::Matrix<double, 1, 1> information(1);
  auto edge = graph_slam->add_room_x_corridor_edge(room.node, x_corridor.node, x_corr_room_meas, information);
  graph_slam->add_robust_kernel(edge, "Huber", 1.0);
}

void NeighbourMapper::factor_x_corridor_x_corridor_constraints(std::unique_ptr<GraphSLAM>& graph_slam, s_graphs::Corridors& x_corridor1, const s_graphs::Corridors& x_corridor2, double x_corr_x_corr_meas) {
  bool neighbour_exists = false;
  for(const auto& x_corridor_neighbour_id : x_corridor1.neighbour_ids) {
    if(x_corridor2.id == x_corridor_neighbour_id) {
      neighbour_exists = true;
      break;
    }
  }

  if(!neighbour_exists) x_corridor1.neighbour_ids.push_back(x_corridor2.id);
  Eigen::Matrix<double, 1, 1> information(1);
  auto edge = graph_slam->add_x_corridor_x_corridor_edge(x_corridor1.node, x_corridor2.node, x_corr_x_corr_meas, information);
  graph_slam->add_robust_kernel(edge, "Huber", 1.0);
}

void NeighbourMapper::factor_x_corridor_y_corridor_constraints(std::unique_ptr<GraphSLAM>& graph_slam, s_graphs::Corridors& x_corridor, const s_graphs::Corridors& y_corridor) {
  bool neighbour_exists = false;
  for(const auto& x_corridor_neighbour_id : x_corridor.neighbour_ids) {
    if(y_corridor.id == x_corridor_neighbour_id) {
      neighbour_exists = true;
      break;
    }
  }
  if(!neighbour_exists) x_corridor.neighbour_ids.push_back(y_corridor.id);
  // TODO:HB analyze if edge between x and y will be useful??
}

void NeighbourMapper::factor_y_corridor_room_constraints(std::unique_ptr<GraphSLAM>& graph_slam, s_graphs::Corridors& y_corridor, const s_graphs::Rooms& room, double y_corr_room_meas) {
  bool neighbour_exists = false;
  for(const auto& y_corridor_neighbour_id : y_corridor.neighbour_ids) {
    if(room.id == y_corridor_neighbour_id) {
      neighbour_exists = true;
      break;
    }
  }
  if(!neighbour_exists) y_corridor.neighbour_ids.push_back(room.id);

  Eigen::Matrix<double, 1, 1> information(1);
  auto edge = graph_slam->add_room_y_corridor_edge(room.node, y_corridor.node, y_corr_room_meas, information);
  graph_slam->add_robust_kernel(edge, "Huber", 1.0);
}

void NeighbourMapper::factor_y_corridor_x_corridor_constraints(std::unique_ptr<GraphSLAM>& graph_slam, s_graphs::Corridors& y_corridor, const s_graphs::Corridors& x_corridor) {
  bool neighbour_exists = false;
  for(const auto& y_corridor_neighbour_id : y_corridor.neighbour_ids) {
    if(x_corridor.id == y_corridor_neighbour_id) {
      neighbour_exists = true;
      break;
    }
  }
  if(!neighbour_exists) y_corridor.neighbour_ids.push_back(x_corridor.id);
  // TODO:HB analyze if edge between x and y will be useful??
}

void NeighbourMapper::factor_y_corridor_y_corridor_constraints(std::unique_ptr<GraphSLAM>& graph_slam, s_graphs::Corridors& y_corridor1, const s_graphs::Corridors& y_corridor2, double y_corr_y_corr_meas) {
  bool neighbour_exists = false;
  for(const auto& y_corridor1_neighbour_id : y_corridor1.neighbour_ids) {
    if(y_corridor2.id == y_corridor1_neighbour_id) {
      neighbour_exists = true;
      break;
    }
  }
  if(!neighbour_exists) y_corridor1.neighbour_ids.push_back(y_corridor2.id);

  Eigen::Matrix<double, 1, 1> information(1);
  auto edge = graph_slam->add_y_corridor_y_corridor_edge(y_corridor1.node, y_corridor2.node, y_corr_y_corr_meas, information);
  graph_slam->add_robust_kernel(edge, "Huber", 1.0);
}

}  // namespace s_graphs