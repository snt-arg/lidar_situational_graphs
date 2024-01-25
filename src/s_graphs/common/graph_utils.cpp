#include "s_graphs/common/graph_utils.hpp"

namespace s_graphs {

void GraphUtils::copy_graph(const std::shared_ptr<GraphSLAM>& covisibility_graph,
                            std::unique_ptr<GraphSLAM>& compressed_graph,
                            const std::map<int, KeyFrame::Ptr>& keyframes) {
  compressed_graph->graph->clear();
  copy_graph_vertices(covisibility_graph, compressed_graph);
  std::vector<g2o::VertexSE3*> filtered_k_vec =
      copy_graph_edges(covisibility_graph, compressed_graph);
  connect_broken_keyframes(
      filtered_k_vec, covisibility_graph, compressed_graph, keyframes);
}

void GraphUtils::copy_graph_vertices(
    const std::shared_ptr<GraphSLAM>& covisibility_graph,
    const std::unique_ptr<GraphSLAM>& compressed_graph) {
  for (g2o::HyperGraph::VertexIDMap::iterator it =
           covisibility_graph->graph->vertices().begin();
       it != covisibility_graph->graph->vertices().end();
       ++it) {
    g2o::OptimizableGraph::Vertex* v = (g2o::OptimizableGraph::Vertex*)(it->second);
    if (compressed_graph->graph->vertex(v->id())) continue;

    g2o::VertexSE3* vertex_se3 = dynamic_cast<g2o::VertexSE3*>(v);
    if (vertex_se3) {
      auto keyframe_vert_data = dynamic_cast<OptimizationData*>(vertex_se3->userData());
      bool marginalized = false;
      if (keyframe_vert_data) {
        keyframe_vert_data->get_marginalized_info(marginalized);
      }

      if (!marginalized) {
        if (compressed_graph->graph->vertex(v->id())) continue;
        auto current_vertex = compressed_graph->copy_se3_node(vertex_se3);
      }
      continue;
    }

    g2o::VertexPlane* vertex_plane = dynamic_cast<g2o::VertexPlane*>(v);
    if (vertex_plane) {
      auto current_vertex = compressed_graph->copy_plane_node(vertex_plane);
      continue;
    }

    g2o::VertexWallXYZ* vertex_wall = dynamic_cast<g2o::VertexWallXYZ*>(v);
    if (vertex_wall) {
      auto current_vertex = compressed_graph->copy_wall_node(vertex_wall);
      continue;
    }

    g2o::VertexFloor* vertex_floor = dynamic_cast<g2o::VertexFloor*>(v);
    if (vertex_floor) {
      auto current_vertex = compressed_graph->copy_floor_node(vertex_floor);
      continue;
    }

    g2o::VertexRoom* vertex_room = dynamic_cast<g2o::VertexRoom*>(v);
    if (vertex_room) {
      auto current_vertex = compressed_graph->copy_room_node(vertex_room);
      continue;
    }
  }
}

std::vector<g2o::VertexSE3*> GraphUtils::copy_graph_edges(
    const std::shared_ptr<GraphSLAM>& covisibility_graph,
    const std::unique_ptr<GraphSLAM>& compressed_graph) {
  std::vector<g2o::VertexSE3*> filtered_k_vec;

  for (g2o::HyperGraph::EdgeSet::iterator it =
           covisibility_graph->graph->edges().begin();
       it != covisibility_graph->graph->edges().end();
       ++it) {
    g2o::OptimizableGraph::Edge* e = (g2o::OptimizableGraph::Edge*)(*it);

    auto found_edge =
        std::find_if(compressed_graph->graph->edges().begin(),
                     compressed_graph->graph->edges().end(),
                     boost::bind(&g2o::HyperGraph::Edge::id, _1) == e->id());

    g2o::EdgeSE3* edge_se3 = dynamic_cast<g2o::EdgeSE3*>(e);
    if (edge_se3) {
      if (!compressed_graph->graph->vertex(edge_se3->vertices()[0]->id())) {
        if (compressed_graph->graph->vertex(edge_se3->vertices()[1]->id())) {
          filtered_k_vec.push_back(
              dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[1]));
        }
        continue;
      }
      if (!compressed_graph->graph->vertex(edge_se3->vertices()[1]->id())) {
        if (compressed_graph->graph->vertex(edge_se3->vertices()[0]->id())) {
          filtered_k_vec.push_back(
              dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[0]));
        }
        continue;
      }

      g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(
          compressed_graph->graph->vertices().at(edge_se3->vertices()[0]->id()));
      g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(
          compressed_graph->graph->vertices().at(edge_se3->vertices()[1]->id()));

      if (found_edge != compressed_graph->graph->edges().end()) continue;

      auto edge = compressed_graph->copy_se3_edge(edge_se3, v1, v2);
      compressed_graph->add_robust_kernel(edge, "Huber", 1.0);
      continue;
    }

    if (found_edge != compressed_graph->graph->edges().end()) continue;
    g2o::EdgeSE3Plane* edge_se3_plane = dynamic_cast<g2o::EdgeSE3Plane*>(e);
    if (edge_se3_plane) {
      if (!compressed_graph->graph->vertex(edge_se3_plane->vertices()[0]->id()))
        continue;
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

    g2o::EdgeWall2Planes* edge_wall_2planes = dynamic_cast<g2o::EdgeWall2Planes*>(e);
    if (edge_wall_2planes) {
      g2o::VertexWallXYZ* v1 =
          dynamic_cast<g2o::VertexWallXYZ*>(compressed_graph->graph->vertices().at(
              edge_wall_2planes->vertices()[0]->id()));
      g2o::VertexPlane* v2 =
          dynamic_cast<g2o::VertexPlane*>(compressed_graph->graph->vertices().at(
              edge_wall_2planes->vertices()[1]->id()));
      g2o::VertexPlane* v3 =
          dynamic_cast<g2o::VertexPlane*>(compressed_graph->graph->vertices().at(
              edge_wall_2planes->vertices()[2]->id()));

      auto edge =
          compressed_graph->copy_wall_2planes_edge(edge_wall_2planes, v1, v2, v3);
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

  return filtered_k_vec;
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
    bool marginalized =
        get_keyframe_marg_data(dynamic_cast<g2o::VertexSE3*>(it->second->node));

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
  compressed_graph->graph->vertex(min_keyframe_id)->setFixed(true);

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

  // connect existing keyframes and connect disconnected keyframes
  connect_broken_keyframes(
      filtered_k_vec, covisibility_graph, compressed_graph, keyframes);

  // loop the fixed keyframe set, make it fixed and copy its edges to the local
  // compressed graph
  fix_and_connect_keyframes(compressed_graph.get(), fixed_keyframes_set);

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

        bool marginalized = get_keyframe_marg_data(dynamic_cast<g2o::VertexSE3*>(
            dynamic_cast<g2o::VertexSE3*>(current_vertex_v1)));

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
                bool marginalized = get_keyframe_marg_data(
                    dynamic_cast<g2o::VertexSE3*>(e_v2->vertices()[0]));

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

void GraphUtils::connect_broken_keyframes(
    std::vector<g2o::VertexSE3*> filtered_k_vec,
    const std::shared_ptr<GraphSLAM>& covisibility_graph,
    const std::unique_ptr<GraphSLAM>& compressed_graph,
    const std::map<int, KeyFrame::Ptr>& keyframes) {
  std::sort(filtered_k_vec.begin(),
            filtered_k_vec.end(),
            [](const g2o::VertexSE3* obj1, const g2o::VertexSE3* obj2) {
              return obj1->id() < obj2->id();
            });

  for (int i = 0; i < filtered_k_vec.size(); i += 2) {
    if (filtered_k_vec.size() <= 1) {
      break;
    }

    std::cout << "keyframes to be connected are: " << filtered_k_vec[i]->id() << " "
              << filtered_k_vec[i + 1]->id() << std::endl;

    auto keyframe = keyframes.find(filtered_k_vec[i + 1]->id());
    auto prev_keyframe = keyframes.find(filtered_k_vec[i]->id());

    /* TODO:HB check if its necessary for summing information mats from se3->plane
     * edges as well */
    Eigen::MatrixXd information = Eigen::MatrixXd::Identity(6, 6);
    auto start_keyframe_it = keyframes.lower_bound(prev_keyframe->second->id());
    auto end_keyframe_it = std::prev(keyframes.upper_bound(keyframe->second->id()));

    for (auto it = start_keyframe_it; it != end_keyframe_it; ++it) {
      auto edges = it->second->node->edges();
      g2o::HyperGraph::EdgeSet tmp(it->second->node->edges());

      for (auto edge_itr = tmp.begin(); edge_itr != tmp.end(); edge_itr++) {
        g2o::HyperGraph::Edge* edge = *edge_itr;

        auto found_edge_itr = std::find_if(
            covisibility_graph->graph->edges().begin(),
            covisibility_graph->graph->edges().end(),
            [edge](const g2o::HyperGraph::Edge* e) { return e->id() == edge->id(); });

        if (found_edge_itr != covisibility_graph->graph->edges().end()) {
          g2o::EdgeSE3* edge_se3 = dynamic_cast<g2o::EdgeSE3*>(*found_edge_itr);
          if (edge_se3) {
            const g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(
                covisibility_graph->graph->vertex(edge_se3->vertices()[0]->id()));

            const g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(
                covisibility_graph->graph->vertex(edge_se3->vertices()[1]->id()));

            if (v2->id() == it->second->node->id() &&
                v1->id() == std::next(it)->second->node->id()) {
              information = information * edge_se3->information();
            }
            continue;
          }
        } else
          continue;
      }
    }

    // find the proper vertex pointer in the compressed graph
    auto vertex1 = dynamic_cast<g2o::VertexSE3*>(
        compressed_graph->graph->vertex(filtered_k_vec[i + 1]->id()));
    auto vertex2 = dynamic_cast<g2o::VertexSE3*>(
        compressed_graph->graph->vertex(filtered_k_vec[i]->id()));

    /* TODO:HB check icp-matching between the keyframes to get proper relative pose
     * and the information matrix */
    Eigen::Isometry3d relative_pose =
        vertex1->estimate().inverse() * vertex2->estimate();

    if (vertex1 && vertex2) {
      /* TODO: HB setting high value for information makes the graph diverge */
      // information.setIdentity();
      information = Eigen::MatrixXd::Identity(6, 6);

      auto edge = compressed_graph->add_se3_edge(
          vertex1, vertex2, relative_pose, information, true);
      compressed_graph->add_robust_kernel(edge, "Huber", 1.0);
      std::cout << "add edge between disconnected keyframes " << std::endl;
    }
  }
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
        continue;
      }
    }
  }
}

void GraphUtils::fix_and_connect_keyframes(
    GraphSLAM* compressed_graph,
    const std::unordered_set<g2o::VertexSE3*>& fixed_keyframes_set) {
  std::set<g2o::VertexSE3*> compressed_graph_keyframes_set;
  for (auto fixed_keyframe : fixed_keyframes_set) {
    auto current_fixed_keyframe = compressed_graph->copy_se3_node(fixed_keyframe);
    current_fixed_keyframe->setFixed(true);
    compressed_graph_keyframes_set.insert(current_fixed_keyframe);
  }

  std::unordered_set<int> added_edge_set;
  std::vector<g2o::VertexSE3*> eliminated_keyframes_vec;
  for (auto fixed_keyframe : fixed_keyframes_set) {
    auto current_fixed_keyframe = dynamic_cast<g2o::VertexSE3*>(
        compressed_graph->graph->vertex(fixed_keyframe->id()));
    bool has_edge_se3 = false;
    for (g2o::HyperGraph::EdgeSet::iterator it = fixed_keyframe->edges().begin();
         it != fixed_keyframe->edges().end();
         ++it) {
      g2o::OptimizableGraph::Edge* e = (g2o::OptimizableGraph::Edge*)(*it);

      g2o::EdgeSE3* edge_se3 = dynamic_cast<g2o::EdgeSE3*>(e);
      if (edge_se3) {
        if (compressed_graph->graph->vertex(edge_se3->vertices()[0]->id()) &&
            edge_se3->vertices()[0]->id() != current_fixed_keyframe->id()) {
          g2o::VertexSE3* current_vertex1 = dynamic_cast<g2o::VertexSE3*>(
              compressed_graph->graph->vertex(edge_se3->vertices()[0]->id()));

          if (added_edge_set.find(edge_se3->id()) == added_edge_set.end()) {
            if (!current_vertex1->fixed()) {
              auto current_edge = compressed_graph->copy_se3_edge(
                  edge_se3, current_vertex1, current_fixed_keyframe);
              compressed_graph->add_robust_kernel(current_edge, "Huber", 1.0);

              added_edge_set.insert(edge_se3->id());
              has_edge_se3 = true;
              continue;
            }
          } else
            has_edge_se3 = true;

        } else if (compressed_graph->graph->vertex(edge_se3->vertices()[1]->id()) &&
                   edge_se3->vertices()[1]->id() != current_fixed_keyframe->id()) {
          g2o::VertexSE3* current_vertex2 = dynamic_cast<g2o::VertexSE3*>(
              compressed_graph->graph->vertex(edge_se3->vertices()[1]->id()));

          if (added_edge_set.find(edge_se3->id()) == added_edge_set.end()) {
            if (!current_vertex2->fixed()) {
              auto current_edge = compressed_graph->copy_se3_edge(
                  edge_se3, current_fixed_keyframe, current_vertex2);
              compressed_graph->add_robust_kernel(current_edge, "Huber", 1.0);

              added_edge_set.insert(edge_se3->id());
              has_edge_se3 = true;
              continue;
            }
          } else
            has_edge_se3 = true;
        }
      }
    }
    // remove se3 vertex if it doesnt have any se3->se3 edge
    // if (!has_edge_se3) {
    //   eliminated_keyframes_vec.push_back(current_fixed_keyframe);
    // }
  }

  // for (auto eliminated_keyframe : eliminated_keyframes_vec) {
  //   compressed_graph->graph->removeVertex(eliminated_keyframe);
  // }

  for (auto fixed_keyframe : fixed_keyframes_set) {
    for (g2o::HyperGraph::EdgeSet::iterator it = fixed_keyframe->edges().begin();
         it != fixed_keyframe->edges().end();
         ++it) {
      g2o::OptimizableGraph::Edge* e = (g2o::OptimizableGraph::Edge*)(*it);
      auto current_fixed_keyframe = dynamic_cast<g2o::VertexSE3*>(
          compressed_graph->graph->vertex(fixed_keyframe->id()));

      if (current_fixed_keyframe) {
        g2o::EdgeSE3Plane* edge_se3_plane = dynamic_cast<g2o::EdgeSE3Plane*>(e);
        if (edge_se3_plane) {
          if (compressed_graph->graph->vertex(edge_se3_plane->vertices()[1]->id())) {
            g2o::VertexPlane* current_vertex2 = dynamic_cast<g2o::VertexPlane*>(
                compressed_graph->graph->vertex(edge_se3_plane->vertices()[1]->id()));

            auto current_edge = compressed_graph->copy_se3_plane_edge(
                edge_se3_plane, current_fixed_keyframe, current_vertex2);
            compressed_graph->add_robust_kernel(current_edge, "Huber", 1.0);
            continue;
          }
        }
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

void GraphUtils::set_marginalize_info(
    const std::shared_ptr<GraphSLAM>& local_graph,
    const std::shared_ptr<GraphSLAM>& covisibility_graph,
    const std::map<int, KeyFrame::Ptr>& room_keyframes) {
  int k_counter = 0;

  for (const auto& keyframe : room_keyframes) {
    g2o::OptimizableGraph::Vertex* local_vertex =
        (g2o::OptimizableGraph::Vertex*)(local_graph->graph->vertex(keyframe.first));
    g2o::VertexSE3* local_vertex_se3 = dynamic_cast<g2o::VertexSE3*>(local_vertex);

    if (local_vertex_se3) {
      auto keyframe_vert_data =
          dynamic_cast<OptimizationData*>(local_vertex_se3->userData());

      if (keyframe_vert_data) {
        bool anchor_node = false;
        keyframe_vert_data->get_anchor_node_info(anchor_node);
        if (anchor_node) continue;
      }

      g2o::OptimizableGraph::Vertex* covis_vertex =
          (g2o::OptimizableGraph::Vertex*)(covisibility_graph->graph->vertex(
              local_vertex_se3->id()));
      g2o::VertexSE3* covis_vertex_se3 = dynamic_cast<g2o::VertexSE3*>(covis_vertex);

      if (covis_vertex_se3) {
        auto current_data =
            dynamic_cast<OptimizationData*>(covis_vertex_se3->userData());
        bool marginalized = false;
        if (current_data) {
          current_data->get_marginalized_info(marginalized);
        }

        if (k_counter == 0) {
          OptimizationData* data = new OptimizationData();
          data->set_rep_node_info(true);
          covis_vertex_se3->setUserData(data);
        } else if (k_counter != 0 && !marginalized) {
          if (!current_data) {
            OptimizationData* data = new OptimizationData();
            marginalized = true;
            data->set_marginalized_info(marginalized);
            covis_vertex_se3->setUserData(data);
            covis_vertex_se3->setEstimate((local_vertex_se3)->estimate());
          }
        }
      }
    }
    k_counter++;
  }

  for (g2o::HyperGraph::VertexIDMap::iterator it =
           local_graph->graph->vertices().begin();
       it != local_graph->graph->vertices().end();
       ++it) {
    g2o::OptimizableGraph::Vertex* v = (g2o::OptimizableGraph::Vertex*)(it->second);
    g2o::VertexPlane* vertex_plane = dynamic_cast<g2o::VertexPlane*>(v);
    if (vertex_plane) {
      // find the equivalent in the covis graph
      auto covis_v = dynamic_cast<g2o::VertexPlane*>(
          covisibility_graph->graph->vertex(vertex_plane->id()));
      if (covis_v) covis_v->setEstimate(vertex_plane->estimate());
      continue;
    }

    g2o::VertexRoom* vertex_room = dynamic_cast<g2o::VertexRoom*>(v);
    if (vertex_room) {
      auto covis_v = dynamic_cast<g2o::VertexRoom*>(
          covisibility_graph->graph->vertex(vertex_room->id()));
      if (covis_v) covis_v->setEstimate(vertex_room->estimate());
      continue;
    }
  }
}

bool GraphUtils::get_keyframe_marg_data(g2o::VertexSE3* vertex_se3) {
  auto current_data = dynamic_cast<OptimizationData*>(vertex_se3->userData());
  bool marginalized = false;
  if (current_data) {
    current_data->get_marginalized_info(marginalized);
  }

  return marginalized;
}

void GraphUtils::set_stair_keyframes(const std::vector<int>& ids,
                                     const std::map<int, KeyFrame::Ptr>& keyframes) {
  for (int id : ids) {
    auto keyframe_vert_data =
        dynamic_cast<OptimizationData*>(keyframes.at(id)->node->userData());
    if (keyframe_vert_data) {
      keyframe_vert_data->set_stair_node_info(true);
    } else {
      OptimizationData* data = new OptimizationData();
      data->set_stair_node_info(true);
      keyframes.at(id)->node->setUserData(data);
    }
  }
}

void GraphUtils::update_node_floor_level(
    const int& last_keyframe_id,
    const int& current_floor_level,
    const std::map<int, KeyFrame::Ptr>& keyframes,
    std::unordered_map<int, VerticalPlanes>& x_vert_planes,
    std::unordered_map<int, VerticalPlanes>& y_vert_planes,
    std::unordered_map<int, Rooms>& rooms_vec,
    std::unordered_map<int, InfiniteRooms>& x_infinite_rooms,
    std::unordered_map<int, InfiniteRooms>& y_infinite_rooms) {
  std::vector<g2o::VertexPlane*> connected_planes;
  auto it = keyframes.find(last_keyframe_id);
  if (it != keyframes.end()) {
    ++it;
    for (; it != keyframes.end(); ++it) {
      // change the floor level to the new floor level
      it->second->floor_level = current_floor_level;

      // get the connected planes and update their floor level
      for (g2o::HyperGraph::EdgeSet::iterator e_it = it->second->node->edges().begin();
           e_it != it->second->node->edges().end();
           ++e_it) {
        g2o::OptimizableGraph::Edge* e = (g2o::OptimizableGraph::Edge*)(*e_it);
        g2o::EdgeSE3Plane* edge_se3_plane = dynamic_cast<g2o::EdgeSE3Plane*>(e);
        if (edge_se3_plane) {
          auto plane = x_vert_planes.find(edge_se3_plane->vertices()[1]->id());
          if (plane != x_vert_planes.end()) {
            plane->second.floor_level = current_floor_level;
            connected_planes.push_back(plane->second.plane_node);
            continue;
          } else {
            auto plane = y_vert_planes.find(edge_se3_plane->vertices()[1]->id());
            if (plane != y_vert_planes.end()) {
              plane->second.floor_level = current_floor_level;
              connected_planes.push_back(plane->second.plane_node);
              continue;
            }
          }
        }
      }
    }

    // loop through the plane nodes to find connected rooms and change their floor level
    for (const auto& connected_plane : connected_planes) {
      for (g2o::HyperGraph::EdgeSet::iterator e_it = connected_plane->edges().begin();
           e_it != connected_plane->edges().end();
           ++e_it) {
        g2o::OptimizableGraph::Edge* e = (g2o::OptimizableGraph::Edge*)(*e_it);
        g2o::EdgeRoom2Planes* edge_room_2planes =
            dynamic_cast<g2o::EdgeRoom2Planes*>(e);
        if (edge_room_2planes) {
          auto inf_room = x_infinite_rooms.find(edge_room_2planes->vertices()[0]->id());
          if (inf_room != x_infinite_rooms.end()) {
            inf_room->second.floor_level = current_floor_level;
            continue;
          } else {
            auto inf_room =
                y_infinite_rooms.find(edge_room_2planes->vertices()[0]->id());
            if (inf_room != y_infinite_rooms.end()) {
              inf_room->second.floor_level = current_floor_level;
              continue;
            }
          }
        }

        g2o::EdgeRoom4Planes* edge_room_4planes =
            dynamic_cast<g2o::EdgeRoom4Planes*>(e);
        if (edge_room_4planes) {
          auto room = rooms_vec.find(edge_room_4planes->vertices()[0]->id());
          if (room != rooms_vec.end()) {
            room->second.floor_level = current_floor_level;
            continue;
          }
        }
      }
    }
  }
}

}  // namespace s_graphs