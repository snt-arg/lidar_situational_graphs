#include "s_graphs/backend/room_graph_generator.hpp"

namespace s_graphs {

RoomGraphGenerator::RoomGraphGenerator(rclcpp::Node::SharedPtr node) {}

RoomGraphGenerator::~RoomGraphGenerator() {}

Rooms RoomGraphGenerator::get_current_room(
    const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    const KeyFrame::Ptr keyframe,
    const std::unordered_map<int, Rooms>& rooms_vec) {
  Rooms current_room;
  for (const auto& room : rooms_vec) {
    if (room.second.floor_level != keyframe->floor_level) continue;
    if (is_keyframe_inside_room(room.second, x_vert_planes, y_vert_planes, keyframe)) {
      current_room = room.second;
      std::cout << "robot is currently in room with pose "
                << room.second.node->estimate().translation() << std::endl;
    }
  }
  return current_room;
}

std::map<int, KeyFrame::Ptr> RoomGraphGenerator::get_keyframes_inside_room(
    const Rooms& current_room,
    const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    const std::map<int, KeyFrame::Ptr>& keyframes) {
  std::map<int, s_graphs::KeyFrame::Ptr> room_keyframes;

  std::map<int, KeyFrame::Ptr> floor_keyframes;
  for (const auto& keyframe : keyframes) {
    if (current_room.floor_level == keyframe.second->floor_level)
      floor_keyframes.insert({keyframe.first, keyframe.second});
  }

  if (current_room.node != nullptr) {
    room_keyframes =
        get_room_keyframes(current_room, x_vert_planes, y_vert_planes, floor_keyframes);
  }

  std::map<int, s_graphs::KeyFrame::Ptr> filtered_room_keyframes;
  for (const auto& room_keyframe : room_keyframes) {
    s_graphs::KeyFrame::Ptr new_room_keyframe =
        std::make_shared<KeyFrame>(*room_keyframe.second);
    filtered_room_keyframes.insert({room_keyframe.first, new_room_keyframe});
  }

  return filtered_room_keyframes;
}

std::vector<const s_graphs::VerticalPlanes*> get_room_planes(
    const Rooms& current_room,
    const std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    const std::unordered_map<int, VerticalPlanes>& y_vert_planes) {
  auto planes = obtain_planes_from_room(current_room, x_vert_planes, y_vert_planes);
  return planes;
}

void RoomGraphGenerator::generate_local_graph(
    std::unique_ptr<KeyframeMapper>& keyframe_mapper,
    std::shared_ptr<GraphSLAM> covisibility_graph,
    std::map<int, KeyFrame::Ptr> filtered_keyframes,
    const Eigen::Isometry3d& odom2map,
    Rooms& current_room) {
  std::deque<KeyFrame::Ptr> new_room_keyframes;

  current_room.local_graph->graph->clear();
  current_room.room_keyframes.clear();

  // check which keyframes already exist in the local graph and add only new ones
  for (const auto& filtered_keyframe : filtered_keyframes) {
    if (current_room.local_graph->graph->vertex(filtered_keyframe.second->id())) {
      continue;

    } else
      new_room_keyframes.push_back(filtered_keyframe.second);
  }

  g2o::VertexSE3* anchor_node;
  g2o::EdgeSE3* anchor_edge;
  keyframe_mapper->map_keyframes(current_room.local_graph,
                                 covisibility_graph,
                                 odom2map,
                                 new_room_keyframes,
                                 current_room.room_keyframes,
                                 anchor_node,
                                 anchor_edge);

  // fixing the first non-marginalized keyframe in the room graph with anchor node
  // Eigen::Isometry3d anchor_target =
  //     static_cast<g2o::VertexSE3*>(anchor_edge->vertices()[1])->estimate();
  // anchor_node->setEstimate(anchor_target);

  // get the edges of the keyframe in the cov graph and add them to the local
  // graph
  std::unordered_set<g2o::VertexSE3*> fixed_keyframes_set =
      GraphUtils::connect_keyframes_planes(covisibility_graph,
                                           current_room.local_graph.get());

  // loop the fixed keyframe set, make it fixed and add copy its edges to the local
  // compressed graph
  GraphUtils::fix_and_connect_keyframes(current_room.local_graph.get(),
                                        fixed_keyframes_set);

  // get the vertices of the planes in the local graph and connect their edges from
  // cov graph to local graph
  GraphUtils::connect_planes_rooms(covisibility_graph, current_room.local_graph.get());
}

void RoomGraphGenerator::update_room_graph(
    const Rooms room,
    const std::shared_ptr<GraphSLAM>& covisibility_graph) {
  // check the difference between the first representative node in covis graph and its
  // corresponding in the room-local graph
  Eigen::Isometry3d delta =
      dynamic_cast<g2o::VertexSE3*>(
          room.local_graph->graph->vertex(room.room_keyframes.begin()->first))
          ->estimate()
          .inverse() *
      dynamic_cast<g2o::VertexSE3*>(
          covisibility_graph->graph->vertex(room.room_keyframes.begin()->first))
          ->estimate();

  g2o::Vector6 delta_vec = g2o::internal::toVectorMQT(delta);
  if (fabs(delta_vec[0]) > 0.1 || fabs(delta_vec[1]) > 0.1 ||
      fabs(delta_vec[2]) > 0.1 || fabs(delta_vec[3]) > 0.1 ||
      fabs(delta_vec[4]) > 0.1 || fabs(delta_vec[5]) > 0.1) {
    for (g2o::HyperGraph::VertexIDMap::iterator it =
             room.local_graph->graph->vertices().begin();
         it != room.local_graph->graph->vertices().end();
         ++it) {
      g2o::OptimizableGraph::Vertex* v = (g2o::OptimizableGraph::Vertex*)(it->second);
      g2o::VertexSE3* vertex_se3 = dynamic_cast<g2o::VertexSE3*>(v);
      if (vertex_se3) {
        // check the difference between this vertex
        vertex_se3->setEstimate(delta * vertex_se3->estimate());
      }
    }
  }
}

}  // namespace s_graphs
