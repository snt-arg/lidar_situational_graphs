#include "s_graphs/common/graph_utils.hpp"

namespace s_graphs {

void GraphUtils::copy_graph(const std::shared_ptr<GraphSLAM>& covisibility_graph,
                            std::unique_ptr<GraphSLAM>& compressed_graph) {
  for (g2o::HyperGraph::VertexIDMap::iterator it =
           covisibility_graph->graph->vertices().begin();
       it != covisibility_graph->graph->vertices().end();
       ++it) {
    g2o::OptimizableGraph::Vertex* v = (g2o::OptimizableGraph::Vertex*)(it->second);
    if (compressed_graph->graph->vertex(v->id())) continue;

    g2o::VertexSE3* vertex_se3 = dynamic_cast<g2o::VertexSE3*>(v);
    if (vertex_se3) {
      auto current_vertex = compressed_graph->copy_se3_node(vertex_se3);
      continue;
    }
    g2o::VertexPlane* vertex_plane = dynamic_cast<g2o::VertexPlane*>(v);
    if (vertex_plane) {
      auto current_vertex = compressed_graph->copy_plane_node(vertex_plane);
      continue;
    }
    g2o::VertexRoom* vertex_room = dynamic_cast<g2o::VertexRoom*>(v);
    if (vertex_room) {
      auto current_vertex = compressed_graph->copy_room_node(vertex_room);
      continue;
    }
    g2o::VertexFloor* vertex_floor = dynamic_cast<g2o::VertexFloor*>(v);
    if (vertex_floor)
      auto current_vertex = compressed_graph->copy_floor_node(vertex_floor);
  }

  for (g2o::HyperGraph::EdgeSet::iterator it =
           covisibility_graph->graph->edges().begin();
       it != covisibility_graph->graph->edges().end();
       ++it) {
    g2o::OptimizableGraph::Edge* e = (g2o::OptimizableGraph::Edge*)(*it);

    auto edge = std::find_if(compressed_graph->graph->edges().begin(),
                             compressed_graph->graph->edges().end(),
                             boost::bind(&g2o::HyperGraph::Edge::id, _1) == e->id());

    if (edge != compressed_graph->graph->edges().end()) continue;

    g2o::EdgeSE3* edge_se3 = dynamic_cast<g2o::EdgeSE3*>(e);
    if (edge_se3) {
      g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(
          compressed_graph->graph->vertices().at(edge_se3->vertices()[0]->id()));
      g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(
          compressed_graph->graph->vertices().at(edge_se3->vertices()[1]->id()));
      auto edge = compressed_graph->copy_se3_edge(edge_se3, v1, v2);
      compressed_graph->add_robust_kernel(edge, "Huber", 1.0);
      continue;
    }

    g2o::EdgeSE3Plane* edge_se3_plane = dynamic_cast<g2o::EdgeSE3Plane*>(e);
    if (edge_se3_plane) {
      g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(
          compressed_graph->graph->vertices().at(edge_se3_plane->vertices()[0]->id()));
      g2o::VertexPlane* v2 = dynamic_cast<g2o::VertexPlane*>(
          compressed_graph->graph->vertices().at(edge_se3_plane->vertices()[1]->id()));
      auto edge = compressed_graph->copy_se3_plane_edge(edge_se3_plane, v1, v2);
      compressed_graph->add_robust_kernel(edge, "Huber", 1.0);
      continue;
    }

    g2o::EdgeRoom2Planes* edge_room_2planes = dynamic_cast<g2o::EdgeRoom2Planes*>(e);
    if (edge_room_2planes) {
      g2o::VertexRoom* v1 =
          dynamic_cast<g2o::VertexRoom*>(compressed_graph->graph->vertices().at(
              edge_room_2planes->vertices()[0]->id()));
      g2o::VertexPlane* v2 =
          dynamic_cast<g2o::VertexPlane*>(compressed_graph->graph->vertices().at(
              edge_room_2planes->vertices()[1]->id()));
      g2o::VertexPlane* v3 =
          dynamic_cast<g2o::VertexPlane*>(compressed_graph->graph->vertices().at(
              edge_room_2planes->vertices()[2]->id()));
      g2o::VertexRoom* v4 =
          dynamic_cast<g2o::VertexRoom*>(compressed_graph->graph->vertices().at(
              edge_room_2planes->vertices()[3]->id()));

      auto edge =
          compressed_graph->copy_room_2planes_edge(edge_room_2planes, v1, v2, v3, v4);
      compressed_graph->add_robust_kernel(edge, "Huber", 1.0);
      continue;
    }

    g2o::EdgeRoom4Planes* edge_room_4planes = dynamic_cast<g2o::EdgeRoom4Planes*>(e);
    if (edge_room_4planes) {
      g2o::VertexRoom* v1 =
          dynamic_cast<g2o::VertexRoom*>(compressed_graph->graph->vertices().at(
              edge_room_4planes->vertices()[0]->id()));
      g2o::VertexPlane* v2 =
          dynamic_cast<g2o::VertexPlane*>(compressed_graph->graph->vertices().at(
              edge_room_4planes->vertices()[1]->id()));
      g2o::VertexPlane* v3 =
          dynamic_cast<g2o::VertexPlane*>(compressed_graph->graph->vertices().at(
              edge_room_4planes->vertices()[2]->id()));
      g2o::VertexPlane* v4 =
          dynamic_cast<g2o::VertexPlane*>(compressed_graph->graph->vertices().at(
              edge_room_4planes->vertices()[3]->id()));
      g2o::VertexPlane* v5 =
          dynamic_cast<g2o::VertexPlane*>(compressed_graph->graph->vertices().at(
              edge_room_4planes->vertices()[4]->id()));
      auto edge = compressed_graph->copy_room_4planes_edge(
          edge_room_4planes, v1, v2, v3, v4, v5);
      compressed_graph->add_robust_kernel(edge, "Huber", 1.0);
      continue;
    }

    g2o::Edge2Planes* edge_2planes = dynamic_cast<g2o::Edge2Planes*>(e);
    if (edge_2planes) {
      g2o::VertexPlane* v1 = dynamic_cast<g2o::VertexPlane*>(
          compressed_graph->graph->vertices().at(edge_2planes->vertices()[0]->id()));
      g2o::VertexPlane* v2 = dynamic_cast<g2o::VertexPlane*>(
          compressed_graph->graph->vertices().at(edge_2planes->vertices()[1]->id()));
      auto edge = compressed_graph->copy_2planes_edge(edge_2planes, v1, v2);
      compressed_graph->add_robust_kernel(edge, "Huber", 1.0);
      continue;
    }

    g2o::EdgeFloorRoom* edge_floor_room = dynamic_cast<g2o::EdgeFloorRoom*>(e);
    if (edge_floor_room) {
      g2o::VertexFloor* v1 = dynamic_cast<g2o::VertexFloor*>(
          compressed_graph->graph->vertices().at(edge_floor_room->vertices()[0]->id()));
      g2o::VertexRoom* v2 = dynamic_cast<g2o::VertexRoom*>(
          compressed_graph->graph->vertices().at(edge_floor_room->vertices()[1]->id()));
      auto edge = compressed_graph->copy_floor_room_edge(edge_floor_room, v1, v2);
      compressed_graph->add_robust_kernel(edge, "Huber", 1.0);
      continue;
    }
  }
}

void GraphUtils::copy_windowed_graph(
    const int window_size,
    const std::shared_ptr<GraphSLAM>& covisibility_graph,
    const std::unique_ptr<GraphSLAM>& compressed_graph,
    const std::map<int, KeyFrame::Ptr>& keyframes) {
  // clear compressed graph
  compressed_graph->graph->clear();

  // create the window of keyframes for optimization
  std::map<int, KeyFrame::Ptr> keyframe_window;
  std::map<int, KeyFrame::Ptr> complete_keyframe_window;
  auto it = keyframes.rbegin();
  for (; it != keyframes.rend() && keyframe_window.size() < window_size; ++it) {
    auto keyframe_vert_data =
        dynamic_cast<OptimizationData*>(it->second->node->userData());
    bool marginalized = false;
    if (keyframe_vert_data) {
      keyframe_vert_data->get_marginalized_info(marginalized);
    }

    if (marginalized) {
      complete_keyframe_window.insert(*it);
      continue;
    }

    keyframe_window.insert(*it);
    complete_keyframe_window.insert(*it);
  }

  // copy keyframes in the window to the compressed graph
  int min_keyframe_id =
      copy_keyframes_to_graph(covisibility_graph, compressed_graph, keyframe_window);

  // get all the filtered keyframes added in the compressed graph
  bool anchor_node_exists = false;
  std::vector<g2o::VertexSE3*> filtered_k_vec =
      connect_keyframes(covisibility_graph,
                        compressed_graph,
                        complete_keyframe_window,
                        anchor_node_exists);

  // check from all the keyframes added in the compressed graph, which have
  // connections to planes
  std::unordered_set<g2o::VertexSE3*> fixed_keyframes_set =
      connect_keyframes_planes(covisibility_graph, compressed_graph.get());

  // check from all the planes added in the compressed graph, which have
  // connections to rooms
  connect_planes_rooms(covisibility_graph, compressed_graph.get());

  // Check from all rooms added to which floor they are connected
  // connect_rooms_floors(covisibility_graph, compressed_graph.get());
}

int GraphUtils::copy_keyframes_to_graph(
    const std::shared_ptr<GraphSLAM>& covisibility_graph,
    const std::unique_ptr<GraphSLAM>& compressed_graph,
    const std::map<int, KeyFrame::Ptr>& keyframe_window) {
  int min_keyframe_id = std::numeric_limits<int>::max();

  for (g2o::HyperGraph::VertexIDMap::iterator it =
           covisibility_graph->graph->vertices().begin();
       it != covisibility_graph->graph->vertices().end();
       ++it) {
    g2o::OptimizableGraph::Vertex* v = (g2o::OptimizableGraph::Vertex*)(it->second);
    if (compressed_graph->graph->vertex(v->id())) continue;

    g2o::VertexSE3* vertex_se3 = dynamic_cast<g2o::VertexSE3*>(v);
    if (vertex_se3) {
      auto it = keyframe_window.find(vertex_se3->id());
      // check if the vertex is in the local keyframe window
      if (it != keyframe_window.end()) {
        if (compressed_graph->graph->vertex(v->id())) continue;
        auto current_vertex = compressed_graph->copy_se3_node(vertex_se3);
        if (min_keyframe_id > current_vertex->id()) {
          min_keyframe_id = current_vertex->id();
        }
      }
    }
  }

  return min_keyframe_id;
}

std::vector<g2o::VertexSE3*> GraphUtils::connect_keyframes(
    const std::shared_ptr<GraphSLAM>& covisibility_graph,
    const std::unique_ptr<GraphSLAM>& compressed_graph,
    const std::map<int, KeyFrame::Ptr>& complete_keyframe_window,
    bool& anchor_node_exists) {
  std::vector<g2o::VertexSE3*> filtered_k_vec;

  for (g2o::HyperGraph::EdgeSet::iterator it =
           covisibility_graph->graph->edges().begin();
       it != covisibility_graph->graph->edges().end();
       ++it) {
    g2o::OptimizableGraph::Edge* e = (g2o::OptimizableGraph::Edge*)(*it);

    auto edge = std::find_if(compressed_graph->graph->edges().begin(),
                             compressed_graph->graph->edges().end(),
                             boost::bind(&g2o::HyperGraph::Edge::id, _1) == e->id());

    if (edge != compressed_graph->graph->edges().end()) continue;

    // g2o::EdgeLoopClosure* edge_loop_closure = dynamic_cast<g2o::EdgeLoopClosure*>(e);
    // if (edge_loop_closure) {
    //   g2o::VertexSE3* current_vertex1 = nullptr;
    //   g2o::VertexSE3* current_vertex2 = nullptr;
    //   if (!compressed_graph->graph->vertex(edge_loop_closure->vertices()[0]->id()) &&
    //       compressed_graph->graph->vertex(edge_loop_closure->vertices()[1]->id())) {
    //     g2o::VertexSE3* v1 =
    //         dynamic_cast<g2o::VertexSE3*>(covisibility_graph->graph->vertices().at(
    //             edge_loop_closure->vertices()[0]->id()));
    //     current_vertex1 = compressed_graph->copy_se3_node(v1);
    //     current_vertex2 = dynamic_cast<g2o::VertexSE3*>(
    //         compressed_graph->graph->vertex(edge_loop_closure->vertices()[1]->id()));

    //   } else if (!compressed_graph->graph->vertex(
    //                  edge_loop_closure->vertices()[1]->id()) &&
    //              compressed_graph->graph->vertex(
    //                  edge_loop_closure->vertices()[0]->id())) {
    //     g2o::VertexSE3* v2 =
    //         dynamic_cast<g2o::VertexSE3*>(covisibility_graph->graph->vertices().at(
    //             edge_loop_closure->vertices()[1]->id()));
    //     current_vertex2 = compressed_graph->copy_se3_node(v2);
    //     current_vertex1 = dynamic_cast<g2o::VertexSE3*>(
    //         compressed_graph->graph->vertex(edge_loop_closure->vertices()[0]->id()));

    //   } else if (compressed_graph->graph->vertex(
    //                  edge_loop_closure->vertices()[1]->id()) &&
    //              compressed_graph->graph->vertex(
    //                  edge_loop_closure->vertices()[1]->id())) {
    //     current_vertex1 = dynamic_cast<g2o::VertexSE3*>(
    //         compressed_graph->graph->vertex(edge_loop_closure->vertices()[0]->id()));
    //     current_vertex2 = dynamic_cast<g2o::VertexSE3*>(
    //         compressed_graph->graph->vertex(edge_loop_closure->vertices()[1]->id()));
    //   } else {
    //     continue;
    //   }

    //   if (current_vertex1 && current_vertex2) {
    //     auto edge = compressed_graph->copy_loop_closure_edge(
    //         edge_loop_closure, current_vertex1, current_vertex2);
    //     compressed_graph->add_robust_kernel(edge, "Huber", 1.0);
    //     edge->setLevel(0);
    //   }

    //   continue;
    // }

    g2o::EdgeSE3* edge_se3 = dynamic_cast<g2o::EdgeSE3*>(e);
    if (edge_se3) {
      if (complete_keyframe_window.count(edge_se3->vertices()[0]->id()) > 0 &&
          complete_keyframe_window.count(edge_se3->vertices()[1]->id()) > 0) {
        if (!compressed_graph->graph->vertex(edge_se3->vertices()[0]->id()) &&
            compressed_graph->graph->vertex(edge_se3->vertices()[1]->id())) {
          filtered_k_vec.push_back(
              dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[1]));

          continue;
        } else if (!compressed_graph->graph->vertex(edge_se3->vertices()[1]->id()) &&
                   compressed_graph->graph->vertex(edge_se3->vertices()[0]->id())) {
          filtered_k_vec.push_back(
              dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[0]));

          continue;
        } else if (compressed_graph->graph->vertex(edge_se3->vertices()[0]->id()) &&
                   compressed_graph->graph->vertex(edge_se3->vertices()[1]->id())) {
          g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(
              compressed_graph->graph->vertices().at(edge_se3->vertices()[0]->id()));
          g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(
              compressed_graph->graph->vertices().at(edge_se3->vertices()[1]->id()));

          if (v1 && v2) {
            auto edge = compressed_graph->copy_se3_edge(edge_se3, v1, v2);
            compressed_graph->add_robust_kernel(edge, "Huber", 1.0);
            edge->setLevel(0);
            continue;
          }
        } else {
          continue;
        }
      }
      // check if atleast of the vertices of the cov edge exist in the keyframe window
      else if (complete_keyframe_window.count(edge_se3->vertices()[0]->id()) > 0 ||
               complete_keyframe_window.count(edge_se3->vertices()[1]->id()) > 0) {
        auto current_vertex1_data = dynamic_cast<OptimizationData*>(
            dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[0])->userData());
        auto current_vertex2_data = dynamic_cast<OptimizationData*>(
            dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[1])->userData());

        bool anchor_node1 = false;
        bool anchor_node2 = false;
        if (current_vertex1_data)
          current_vertex1_data->get_anchor_node_info(anchor_node1);
        if (current_vertex2_data)
          current_vertex2_data->get_anchor_node_info(anchor_node2);

        // this is anchor node and its edge
        if (anchor_node1) {
          anchor_node_exists = true;
          auto anchor_vertex = compressed_graph->copy_se3_node(
              dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[0]));
          auto current_vertex = dynamic_cast<g2o::VertexSE3*>(
              compressed_graph->graph->vertex(edge_se3->vertices()[1]->id()));

          auto current_edge =
              compressed_graph->copy_se3_edge(edge_se3, anchor_vertex, current_vertex);
          compressed_graph->add_robust_kernel(current_edge, "Huber", 1.0);
          current_edge->setLevel(0);
          continue;
        } else if (anchor_node2) {
          anchor_node_exists = true;
          auto current_vertex = dynamic_cast<g2o::VertexSE3*>(
              compressed_graph->graph->vertex(edge_se3->vertices()[0]->id()));
          auto anchor_vertex = compressed_graph->copy_se3_node(
              dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[1]));

          auto current_edge =
              compressed_graph->copy_se3_edge(edge_se3, current_vertex, anchor_vertex);
          compressed_graph->add_robust_kernel(current_edge, "Huber", 1.0);
          current_edge->setLevel(0);
          continue;
        }
      }
    }
  }

  return filtered_k_vec;
}

std::unordered_set<g2o::VertexSE3*> GraphUtils::connect_keyframes_planes(
    const std::shared_ptr<GraphSLAM>& covisibility_graph,
    GraphSLAM* compressed_graph) {
  std::unordered_set<g2o::VertexSE3*> fixed_keyframes_set;
  for (g2o::HyperGraph::EdgeSet::iterator it =
           covisibility_graph->graph->edges().begin();
       it != covisibility_graph->graph->edges().end();
       ++it) {
    g2o::OptimizableGraph::Edge* e = (g2o::OptimizableGraph::Edge*)(*it);

    g2o::EdgeSE3Plane* edge_se3_plane = dynamic_cast<g2o::EdgeSE3Plane*>(e);
    if (edge_se3_plane) {
      g2o::VertexSE3* v1 =
          dynamic_cast<g2o::VertexSE3*>(covisibility_graph->graph->vertices().at(
              edge_se3_plane->vertices()[0]->id()));

      g2o::VertexSE3* current_vertex_v1 =
          dynamic_cast<g2o::VertexSE3*>(compressed_graph->graph->vertex(v1->id()));

      if (current_vertex_v1) {
        g2o::VertexPlane* v2 =
            dynamic_cast<g2o::VertexPlane*>(covisibility_graph->graph->vertices().at(
                edge_se3_plane->vertices()[1]->id()));

        g2o::VertexPlane* current_vertex_v2;
        if (!compressed_graph->graph->vertex(v2->id())) {
          current_vertex_v2 = compressed_graph->copy_plane_node(v2);
        } else if (compressed_graph->graph->vertex(v2->id())) {
          current_vertex_v2 = dynamic_cast<g2o::VertexPlane*>(
              compressed_graph->graph->vertex(v2->id()));
        } else
          continue;

        auto edge = compressed_graph->copy_se3_plane_edge(
            edge_se3_plane, current_vertex_v1, current_vertex_v2);
        compressed_graph->add_robust_kernel(edge, "Huber", 1.0);

        bool marginalized = false;
        auto keyframe_vert_data = dynamic_cast<OptimizationData*>(
            dynamic_cast<g2o::VertexSE3*>(current_vertex_v1)->userData());

        if (keyframe_vert_data) {
          keyframe_vert_data->get_marginalized_info(marginalized);
        }

        if (!marginalized)
          edge->setLevel(0);
        else
          edge->setLevel(1);

        // get all the keyframes connected with this plane vertex
        for (g2o::HyperGraph::EdgeSet::iterator it_v2 = v2->edges().begin();
             it_v2 != v2->edges().end();
             ++it_v2) {
          g2o::OptimizableGraph::Edge* e_v2 = (g2o::OptimizableGraph::Edge*)(*it_v2);
          g2o::EdgeSE3Plane* edge_v2_se3 = dynamic_cast<g2o::EdgeSE3Plane*>(e_v2);

          if (edge_v2_se3) {
            if (!compressed_graph->graph->vertex(e_v2->vertices()[0]->id())) {
              if (fixed_keyframes_set.find(dynamic_cast<g2o::VertexSE3*>(
                      e_v2->vertices()[0])) == fixed_keyframes_set.end()) {
                bool marginalized = false;
                auto keyframe_vert_data = dynamic_cast<OptimizationData*>(
                    dynamic_cast<g2o::VertexSE3*>(e_v2->vertices()[0])->userData());

                if (keyframe_vert_data) {
                  keyframe_vert_data->get_marginalized_info(marginalized);
                }
                if (!marginalized) {
                  fixed_keyframes_set.insert(
                      dynamic_cast<g2o::VertexSE3*>(e_v2->vertices()[0]));
                }
              }
            }
          }
        }
      }
    }
  }

  return fixed_keyframes_set;
}

void GraphUtils::connect_planes_rooms(
    const std::shared_ptr<GraphSLAM>& covisibility_graph,
    GraphSLAM* compressed_graph) {
  for (g2o::HyperGraph::EdgeSet::iterator it =
           covisibility_graph->graph->edges().begin();
       it != covisibility_graph->graph->edges().end();
       ++it) {
    g2o::OptimizableGraph::Edge* e = (g2o::OptimizableGraph::Edge*)(*it);

    // g2o::Edge2Planes* edge_2planes = dynamic_cast<g2o::Edge2Planes*>(e);
    // if (edge_2planes) {
    //   g2o::VertexPlane* v1 = dynamic_cast<g2o::VertexPlane*>(
    //       covisibility_graph->graph->vertices().at(edge_2planes->vertices()[0]->id()));

    //   g2o::VertexPlane* current_vertex_v1 =
    //       dynamic_cast<g2o::VertexPlane*>(compressed_graph->graph->vertex(v1->id()));

    //   g2o::VertexPlane* v2 = dynamic_cast<g2o::VertexPlane*>(
    //       covisibility_graph->graph->vertices().at(edge_2planes->vertices()[1]->id()));

    //   g2o::VertexPlane* current_vertex_v2 =
    //       dynamic_cast<g2o::VertexPlane*>(compressed_graph->graph->vertex(v2->id()));

    //   if (current_vertex_v1 && current_vertex_v2) {
    //     auto edge = compressed_graph->copy_2planes_edge(
    //         edge_2planes, current_vertex_v1, current_vertex_v2);
    //     compressed_graph->add_robust_kernel(edge, "Huber", 1.0);
    //     edge->setLevel(0);
    //   }
    // }

    g2o::EdgeRoom4Planes* edge_room_4planes = dynamic_cast<g2o::EdgeRoom4Planes*>(e);
    if (edge_room_4planes) {
      g2o::VertexRoom* rv1 =
          dynamic_cast<g2o::VertexRoom*>(covisibility_graph->graph->vertices().at(
              edge_room_4planes->vertices()[0]->id()));

      g2o::VertexPlane* v1 =
          dynamic_cast<g2o::VertexPlane*>(covisibility_graph->graph->vertices().at(
              edge_room_4planes->vertices()[1]->id()));

      g2o::VertexPlane* current_vertex_v1 =
          dynamic_cast<g2o::VertexPlane*>(compressed_graph->graph->vertex(v1->id()));

      g2o::VertexPlane* v2 =
          dynamic_cast<g2o::VertexPlane*>(covisibility_graph->graph->vertices().at(
              edge_room_4planes->vertices()[2]->id()));

      g2o::VertexPlane* current_vertex_v2 =
          dynamic_cast<g2o::VertexPlane*>(compressed_graph->graph->vertex(v2->id()));

      g2o::VertexPlane* v3 =
          dynamic_cast<g2o::VertexPlane*>(covisibility_graph->graph->vertices().at(
              edge_room_4planes->vertices()[3]->id()));

      g2o::VertexPlane* current_vertex_v3 =
          dynamic_cast<g2o::VertexPlane*>(compressed_graph->graph->vertex(v3->id()));

      g2o::VertexPlane* v4 =
          dynamic_cast<g2o::VertexPlane*>(covisibility_graph->graph->vertices().at(
              edge_room_4planes->vertices()[4]->id()));

      g2o::VertexPlane* current_vertex_v4 =
          dynamic_cast<g2o::VertexPlane*>(compressed_graph->graph->vertex(v4->id()));

      if (current_vertex_v1 && current_vertex_v2 && current_vertex_v3 &&
          current_vertex_v4) {
        g2o::VertexRoom* current_vertex_rv1;
        if (!compressed_graph->graph->vertex(rv1->id())) {
          current_vertex_rv1 = compressed_graph->copy_room_node(rv1);
        } else if (compressed_graph->graph->vertex(rv1->id())) {
          current_vertex_rv1 = dynamic_cast<g2o::VertexRoom*>(
              compressed_graph->graph->vertex(rv1->id()));
        } else
          continue;

        auto edge = compressed_graph->copy_room_4planes_edge(edge_room_4planes,
                                                             current_vertex_rv1,
                                                             current_vertex_v1,
                                                             current_vertex_v2,
                                                             current_vertex_v3,
                                                             current_vertex_v4);
        compressed_graph->add_robust_kernel(edge, "Huber", 1.0);
        edge->setLevel(0);
        continue;
      }
    }

    g2o::EdgeRoom2Planes* edge_room_2planes = dynamic_cast<g2o::EdgeRoom2Planes*>(e);
    if (edge_room_2planes) {
      g2o::VertexRoom* rv1 =
          dynamic_cast<g2o::VertexRoom*>(covisibility_graph->graph->vertices().at(
              edge_room_2planes->vertices()[0]->id()));

      g2o::VertexPlane* v1 =
          dynamic_cast<g2o::VertexPlane*>(covisibility_graph->graph->vertices().at(
              edge_room_2planes->vertices()[1]->id()));

      g2o::VertexPlane* current_vertex_v1 =
          dynamic_cast<g2o::VertexPlane*>(compressed_graph->graph->vertex(v1->id()));

      g2o::VertexPlane* v2 =
          dynamic_cast<g2o::VertexPlane*>(covisibility_graph->graph->vertices().at(
              edge_room_2planes->vertices()[2]->id()));

      g2o::VertexPlane* current_vertex_v2 =
          dynamic_cast<g2o::VertexPlane*>(compressed_graph->graph->vertex(v2->id()));

      g2o::VertexRoom* rv2 =
          dynamic_cast<g2o::VertexRoom*>(covisibility_graph->graph->vertices().at(
              edge_room_2planes->vertices()[3]->id()));

      if (current_vertex_v1 && current_vertex_v2) {
        g2o::VertexRoom *current_vertex_rv1, *current_vertex_rv2;
        if (!compressed_graph->graph->vertex(rv1->id())) {
          current_vertex_rv1 = compressed_graph->copy_room_node(rv1);
        } else if (compressed_graph->graph->vertex(rv1->id())) {
          current_vertex_rv1 = dynamic_cast<g2o::VertexRoom*>(
              compressed_graph->graph->vertex(rv2->id()));
        } else
          continue;

        if (!compressed_graph->graph->vertex(rv2->id())) {
          current_vertex_rv2 = compressed_graph->copy_room_node(rv2);
        } else if (compressed_graph->graph->vertex(rv1->id())) {
          current_vertex_rv2 = dynamic_cast<g2o::VertexRoom*>(
              compressed_graph->graph->vertex(rv2->id()));
        } else
          continue;

        current_vertex_rv2->setFixed(true);
        auto edge = compressed_graph->copy_room_2planes_edge(edge_room_2planes,
                                                             current_vertex_rv1,
                                                             current_vertex_v1,
                                                             current_vertex_v2,
                                                             current_vertex_rv2);
        compressed_graph->add_robust_kernel(edge, "Huber", 1.0);
        edge->setLevel(0);
        continue;
      }
    }
  }
}

void GraphUtils::connect_rooms_floors(
    const std::shared_ptr<GraphSLAM>& covisibility_graph,
    GraphSLAM* compressed_graph) {
  for (g2o::HyperGraph::EdgeSet::iterator it =
           covisibility_graph->graph->edges().begin();
       it != covisibility_graph->graph->edges().end();
       ++it) {
    g2o::OptimizableGraph::Edge* e = (g2o::OptimizableGraph::Edge*)(*it);
    g2o::EdgeFloorRoom* edge_floor_room = dynamic_cast<g2o::EdgeFloorRoom*>(e);
    if (edge_floor_room) {
      g2o::VertexRoom* rv2 =
          dynamic_cast<g2o::VertexRoom*>(covisibility_graph->graph->vertices().at(
              edge_floor_room->vertices()[1]->id()));

      auto current_vertex_v2 =
          dynamic_cast<g2o::VertexRoom*>(compressed_graph->graph->vertex(rv2->id()));

      if (current_vertex_v2) {
        g2o::VertexFloor* current_vertex_v1;
        if (!compressed_graph->graph->vertex(edge_floor_room->vertices()[0]->id())) {
          g2o::VertexFloor* vertex_floor =
              dynamic_cast<g2o::VertexFloor*>(covisibility_graph->graph->vertices().at(
                  edge_floor_room->vertices()[0]->id()));
          current_vertex_v1 = compressed_graph->copy_floor_node(vertex_floor);
        } else if (compressed_graph->graph->vertex(
                       edge_floor_room->vertices()[0]->id())) {
          current_vertex_v1 = dynamic_cast<g2o::VertexFloor*>(
              compressed_graph->graph->vertex(edge_floor_room->vertices()[0]->id()));
        } else
          continue;
        auto edge = compressed_graph->copy_floor_room_edge(
            edge_floor_room, current_vertex_v1, current_vertex_v2);
        compressed_graph->add_robust_kernel(edge, "Huber", 1.0);
        edge->setLevel(0);
      }
    }
  }
}

void GraphUtils::update_graph(const std::unique_ptr<GraphSLAM>& compressed_graph,
                              std::map<int, KeyFrame::Ptr> keyframes,
                              std::unordered_map<int, VerticalPlanes>& x_vert_planes,
                              std::unordered_map<int, VerticalPlanes>& y_vert_planes,
                              std::unordered_map<int, Rooms>& rooms_vec,
                              std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
                              std::unordered_map<int, InfiniteRooms>& y_infinite_rooms,
                              std::unordered_map<int, Floors>& floors_vec) {
  // Loop over all the vertices of the graph
  for (auto it = compressed_graph->graph->vertices().begin();
       it != compressed_graph->graph->vertices().end();
       ++it) {
    g2o::OptimizableGraph::Vertex* v = (g2o::OptimizableGraph::Vertex*)(it->second);
    g2o::VertexSE3* vertex_se3 = dynamic_cast<g2o::VertexSE3*>(v);

    // if vertex is se3 check for it in keyframes vector and update its node estimate
    if (vertex_se3) {
      int id = vertex_se3->id();
      auto keyframe = keyframes.find(id);

      if (keyframe != keyframes.end())
        (*keyframe).second->node->setEstimate(vertex_se3->estimate());
      continue;
    }

    g2o::VertexPlane* vertex_plane = dynamic_cast<g2o::VertexPlane*>(v);
    if (vertex_plane) {
      int id = vertex_plane->id();
      auto x_plane = x_vert_planes.find(id);

      if (x_plane != x_vert_planes.end()) {
        (*x_plane).second.plane_node->setEstimate(vertex_plane->estimate());
        continue;
      } else {
        auto y_plane = y_vert_planes.find(id);

        if (y_plane != y_vert_planes.end()) {
          (*y_plane).second.plane_node->setEstimate(vertex_plane->estimate());
          continue;
        }
      }
    }

    g2o::VertexRoom* vertex_room = dynamic_cast<g2o::VertexRoom*>(v);
    if (vertex_room) {
      int id = vertex_room->id();

      auto room = rooms_vec.find(id);
      if (room != rooms_vec.end()) {
        (*room).second.node->setEstimate(vertex_room->estimate());
        continue;
      } else {
        auto x_inf_room = x_infinite_rooms.find(id);

        if (x_inf_room != x_infinite_rooms.end()) {
          (*x_inf_room).second.node->setEstimate(vertex_room->estimate());
          continue;
        } else {
          auto y_inf_room = y_infinite_rooms.find(id);

          if (y_inf_room != y_infinite_rooms.end()) {
            (*y_inf_room).second.node->setEstimate(vertex_room->estimate());
            continue;
          }
        }
      }
    }

    g2o::VertexFloor* vertex_floor = dynamic_cast<g2o::VertexFloor*>(v);
    if (vertex_floor) {
      int id = vertex_floor->id();
      auto floor = floors_vec.find(id);

      if (floor != floors_vec.end()) {
        (*floor).second.node->setEstimate(vertex_floor->estimate());
        continue;
      }
    }
  }
}

}  // namespace s_graphs