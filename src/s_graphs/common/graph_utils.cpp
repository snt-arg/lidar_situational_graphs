#include "s_graphs/common/graph_utils.hpp"

namespace s_graphs {

void GraphUtils::copy_graph(const std::shared_ptr<GraphSLAM>& covisibility_graph,
                            std::unique_ptr<GraphSLAM>& global_graph) {
  for (g2o::HyperGraph::VertexIDMap::iterator it =
           covisibility_graph->graph->vertices().begin();
       it != covisibility_graph->graph->vertices().end();
       ++it) {
    g2o::OptimizableGraph::Vertex* v = (g2o::OptimizableGraph::Vertex*)(it->second);
    if (global_graph->graph->vertex(v->id())) continue;

    g2o::VertexSE3* vertex_se3 = dynamic_cast<g2o::VertexSE3*>(v);
    if (vertex_se3) {
      auto current_vertex = global_graph->copy_se3_node(vertex_se3);
      continue;
    }
    g2o::VertexPlane* vertex_plane = dynamic_cast<g2o::VertexPlane*>(v);
    if (vertex_plane) {
      auto current_vertex = global_graph->copy_plane_node(vertex_plane);
      continue;
    }
    g2o::VertexRoom* vertex_room = dynamic_cast<g2o::VertexRoom*>(v);
    if (vertex_room) {
      auto current_vertex = global_graph->copy_room_node(vertex_room);
      continue;
    }
    g2o::VertexFloor* vertex_floor = dynamic_cast<g2o::VertexFloor*>(v);
    if (vertex_floor) auto current_vertex = global_graph->copy_floor_node(vertex_floor);
  }

  for (g2o::HyperGraph::EdgeSet::iterator it =
           covisibility_graph->graph->edges().begin();
       it != covisibility_graph->graph->edges().end();
       ++it) {
    g2o::OptimizableGraph::Edge* e = (g2o::OptimizableGraph::Edge*)(*it);

    auto edge = std::find_if(global_graph->graph->edges().begin(),
                             global_graph->graph->edges().end(),
                             boost::bind(&g2o::HyperGraph::Edge::id, _1) == e->id());

    if (edge != global_graph->graph->edges().end()) continue;

    g2o::EdgeSE3* edge_se3 = dynamic_cast<g2o::EdgeSE3*>(e);
    if (edge_se3) {
      g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(
          global_graph->graph->vertices().at(edge_se3->vertices()[0]->id()));
      g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(
          global_graph->graph->vertices().at(edge_se3->vertices()[1]->id()));
      auto edge = global_graph->copy_se3_edge(edge_se3, v1, v2);
      global_graph->add_robust_kernel(edge, "Huber", 1.0);
      continue;
    }

    g2o::EdgeSE3Plane* edge_se3_plane = dynamic_cast<g2o::EdgeSE3Plane*>(e);
    if (edge_se3_plane) {
      g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(
          global_graph->graph->vertices().at(edge_se3_plane->vertices()[0]->id()));
      g2o::VertexPlane* v2 = dynamic_cast<g2o::VertexPlane*>(
          global_graph->graph->vertices().at(edge_se3_plane->vertices()[1]->id()));
      auto edge = global_graph->copy_se3_plane_edge(edge_se3_plane, v1, v2);
      global_graph->add_robust_kernel(edge, "Huber", 1.0);
      continue;
    }

    g2o::EdgeRoom2Planes* edge_room_2planes = dynamic_cast<g2o::EdgeRoom2Planes*>(e);
    if (edge_room_2planes) {
      g2o::VertexRoom* v1 = dynamic_cast<g2o::VertexRoom*>(
          global_graph->graph->vertices().at(edge_room_2planes->vertices()[0]->id()));
      g2o::VertexPlane* v2 = dynamic_cast<g2o::VertexPlane*>(
          global_graph->graph->vertices().at(edge_room_2planes->vertices()[1]->id()));
      g2o::VertexPlane* v3 = dynamic_cast<g2o::VertexPlane*>(
          global_graph->graph->vertices().at(edge_room_2planes->vertices()[2]->id()));
      g2o::VertexRoom* v4 = dynamic_cast<g2o::VertexRoom*>(
          global_graph->graph->vertices().at(edge_room_2planes->vertices()[3]->id()));

      auto edge =
          global_graph->copy_room_2planes_edge(edge_room_2planes, v1, v2, v3, v4);
      global_graph->add_robust_kernel(edge, "Huber", 1.0);
      continue;
    }

    g2o::EdgeRoom4Planes* edge_room_4planes = dynamic_cast<g2o::EdgeRoom4Planes*>(e);
    if (edge_room_4planes) {
      g2o::VertexRoom* v1 = dynamic_cast<g2o::VertexRoom*>(
          global_graph->graph->vertices().at(edge_room_4planes->vertices()[0]->id()));
      g2o::VertexPlane* v2 = dynamic_cast<g2o::VertexPlane*>(
          global_graph->graph->vertices().at(edge_room_4planes->vertices()[1]->id()));
      g2o::VertexPlane* v3 = dynamic_cast<g2o::VertexPlane*>(
          global_graph->graph->vertices().at(edge_room_4planes->vertices()[2]->id()));
      g2o::VertexPlane* v4 = dynamic_cast<g2o::VertexPlane*>(
          global_graph->graph->vertices().at(edge_room_4planes->vertices()[3]->id()));
      g2o::VertexPlane* v5 = dynamic_cast<g2o::VertexPlane*>(
          global_graph->graph->vertices().at(edge_room_4planes->vertices()[4]->id()));
      auto edge =
          global_graph->copy_room_4planes_edge(edge_room_4planes, v1, v2, v3, v4, v5);
      global_graph->add_robust_kernel(edge, "Huber", 1.0);
      continue;
    }

    g2o::Edge2Planes* edge_2planes = dynamic_cast<g2o::Edge2Planes*>(e);
    if (edge_2planes) {
      g2o::VertexPlane* v1 = dynamic_cast<g2o::VertexPlane*>(
          global_graph->graph->vertices().at(edge_2planes->vertices()[0]->id()));
      g2o::VertexPlane* v2 = dynamic_cast<g2o::VertexPlane*>(
          global_graph->graph->vertices().at(edge_2planes->vertices()[1]->id()));
      auto edge = global_graph->copy_2planes_edge(edge_2planes, v1, v2);
      global_graph->add_robust_kernel(edge, "Huber", 1.0);
      continue;
    }

    g2o::EdgeFloorRoom* edge_floor_room = dynamic_cast<g2o::EdgeFloorRoom*>(e);
    if (edge_floor_room) {
      g2o::VertexFloor* v1 = dynamic_cast<g2o::VertexFloor*>(
          global_graph->graph->vertices().at(edge_floor_room->vertices()[0]->id()));
      g2o::VertexRoom* v2 = dynamic_cast<g2o::VertexRoom*>(
          global_graph->graph->vertices().at(edge_floor_room->vertices()[1]->id()));
      auto edge = global_graph->copy_floor_room_edge(edge_floor_room, v1, v2);
      global_graph->add_robust_kernel(edge, "Huber", 1.0);
      continue;
    }
  }
}

void GraphUtils::update_graph(const std::unique_ptr<GraphSLAM>& global_graph,
                              std::vector<KeyFrame::Ptr> keyframes,
                              std::vector<VerticalPlanes>& x_vert_planes,
                              std::vector<VerticalPlanes>& y_vert_planes,
                              std::vector<Rooms>& rooms_vec,
                              std::vector<InfiniteRooms>& x_infinite_rooms,
                              std::vector<InfiniteRooms>& y_infinite_rooms,
                              std::vector<Floors>& floors_vec) {
  // Loop over all the vertices of the graph
  for (auto it = global_graph->graph->vertices().begin();
       it != global_graph->graph->vertices().end();
       ++it) {
    g2o::OptimizableGraph::Vertex* v = (g2o::OptimizableGraph::Vertex*)(it->second);
    g2o::VertexSE3* vertex_se3 = dynamic_cast<g2o::VertexSE3*>(v);

    // if vertex is se3 check for it in keyframes vector and update its node estimate
    if (vertex_se3) {
      int id = vertex_se3->id();
      auto keyframe =
          std::find_if(keyframes.begin(), keyframes.end(), [id](auto& keyframe) {
            return keyframe->id() == id;
          });
      if (keyframe != keyframes.end())
        (*keyframe)->node->setEstimate(vertex_se3->estimate());
      continue;
    }

    g2o::VertexPlane* vertex_plane = dynamic_cast<g2o::VertexPlane*>(v);
    if (vertex_plane) {
      int id = vertex_plane->id();
      auto x_plane = std::find_if(x_vert_planes.begin(),
                                  x_vert_planes.end(),
                                  boost::bind(&VerticalPlanes::id, _1) == id);
      if (x_plane != x_vert_planes.end()) {
        (*x_plane).plane_node->setEstimate(vertex_plane->estimate());
        continue;
      } else {
        auto y_plane = std::find_if(y_vert_planes.begin(),
                                    y_vert_planes.end(),
                                    boost::bind(&VerticalPlanes::id, _1) == id);
        if (y_plane != y_vert_planes.end()) {
          (*y_plane).plane_node->setEstimate(vertex_plane->estimate());
          continue;
        }
      }
    }

    g2o::VertexRoom* vertex_room = dynamic_cast<g2o::VertexRoom*>(v);
    if (vertex_room) {
      int id = vertex_room->id();

      auto room = std::find_if(
          rooms_vec.begin(), rooms_vec.end(), boost::bind(&Rooms::id, _1) == id);
      if (room != rooms_vec.end()) {
        (*room).node->setEstimate(vertex_room->estimate());
        continue;
      } else {
        auto x_inf_room = std::find_if(x_infinite_rooms.begin(),
                                       x_infinite_rooms.end(),
                                       boost::bind(&InfiniteRooms::id, _1) == id);
        if (x_inf_room != x_infinite_rooms.end()) {
          (*x_inf_room).node->setEstimate(vertex_room->estimate());
          continue;
        } else {
          auto y_inf_room = std::find_if(y_infinite_rooms.begin(),
                                         y_infinite_rooms.end(),
                                         boost::bind(&InfiniteRooms::id, _1) == id);
          if (y_inf_room != y_infinite_rooms.end()) {
            (*y_inf_room).node->setEstimate(vertex_room->estimate());
            continue;
          }
        }
      }
    }

    g2o::VertexFloor* vertex_floor = dynamic_cast<g2o::VertexFloor*>(v);
    if (vertex_floor) {
      int id = vertex_floor->id();
      auto floor = std::find_if(
          floors_vec.begin(), floors_vec.end(), boost::bind(&Floors::id, _1) == id);

      if (floor != floors_vec.end()) {
        (*floor).node->setEstimate(vertex_floor->estimate());
        continue;
      }
    }
  }
}

}  // namespace s_graphs