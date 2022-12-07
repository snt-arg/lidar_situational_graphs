# Naming conventions

## RoomAnalyzer

| Current                         | Change To                 |
| ------------------------------- | ------------------------- |
| analyze_skeleton_graph          |                           |
| get_cloud_clusters              | extract_cloud_clusters    |
| get_connected_graph             | extract_connected_graph   |
| get_makerarray_clusters         | extract_marker_array      |
| get_room_planes                 | extract_room_planes       |
| get_convex_hull                 | extract_convex_hull       |
| get_cluster_endpoints           | extract_cluster_endpoints |
| get_centroid_location           | extract_centroid_location |
| get_room_length                 | extract_room_lenght       |
| get_infinite_room_center             | extract_infinite_room_center   |
| perform_room_segmentation       |                           |
| downsample_cloud                |                           |
| check_x1yplane_alignment        | is_x1yplane_aligned       |
| check_x2yplane_alignment        | is_x2yplane_aligned       |
| check_y1xplane_alignment        | is_y2xplane_aligned       |
| check_y2xplane_alignment        | is_y2xplane_aligned       |
| nh                              |                           |
| plane_utils                     |                           |
| vertex_neigh_thres              |                           |
| cloud_clusters                  |                           |
| connected_subgraphs             |                           |
| connected_clusters_marker_array |                           |
| find_plane_points               |                           |
| compute_centroid                |                           |

## RoomMapper

| Current                            | Change To |
| ---------------------------------- | --------- |
| point_difference                   |           |
| parallel_plane_constraint          |           |
| perpendicular_plane_constraint     |           |
| nh                                 |           |
| plane_utils                        |           |
| lookup_infinite_rooms                   |           |
| lookup_infinite_rooms                   |           |
| infinite_room_measurement               |           |
| infinite_room_measurement               |           |
| sort_infinite_rooms                     |           |
| refine_infinite_rooms                   |           |
| factor_infinite_rooms                   |           |
| associate_infinite_rooms                |           |
| associate_infinite_rooms                |           |
| check_infinite_room_ids                 |           |
| parallel_plane_constraint          |           |
| infinite_room_point_diff_threshold      |           |
| infinite_room_max_width                 |           |
| infinite_room_information               |           |
| infinite_room_dist_threshold            |           |
| infinite_room_min_seg_dist              |           |
| use_perpendicular_plane_constraint |           |
| PointNormal                        |           |
| nh                                 |           |
| plane_utils                        |           |
| lookup_rooms                       |           |
| lookup_rooms                       |           |
| room_measurement                   |           |
| sort_rooms                         |           |
| refine_rooms                       |           |
| factor_rooms                       |           |
| associate_rooms                    |           |
| check_room_ids                     |           |
| map_room_from_existing_infinite_rooms   |           |
| map_room_from_existing_x_infinite_room  |           |
| map_room_from_existing_y_infinite_room  |           |
| room_width_diff_threshold          |           |
| room_point_diff_threshold          |           |
| room_max_width                     |           |
| room_information                   |           |
| room_dist_threshold                |           |
| use_perpendicular_plane_constraint |           |

## FloorAnalyzer

| Current                    | Change To |
| -------------------------- | --------- |
| perform_floor_segmentation |           |
| plane_utils                |           |

## FloorMapper

| Current                 | Change To |
| ----------------------- | --------- |
| lookup_floors           |           |
| factor_floor_node       |           |
| update_floor_node       |           |
| factor_floor_room_nodes |           |
| remove_floor_room_nodes |           |
| nh                      |           |
| plane_utils             |           |

## FloorAnalyzer

| Current                        | Change To |
| ------------------------------ | --------- |
| num_vertices                   |           |
| num_edges                      |           |
| num_vertices_local             |           |
| num_edges_local                |           |
| set_solver                     |           |
| add_se3_node                   |           |
| add_plane_node                 |           |
| remove_plnae_node              |           |
| add_point_xyz_node             |           |
| add_infinite_room_node              |           |
| add_room_node                  |           |
| add_floor_node                 |           |
| update_floor_node              |           |
| add_se3_edge                   |           |
| add_se3_plane_edge             |           |
| remove_se3_plane_edge          |           |
| add_se3_point_to_plane_edge    |           |
| add_se3_point_xyz_edge         |           |
| add_plane_normal_prior_edge    |           |
| add_plane_distance_prior_edge  |           |
| add_se3_prior_xy_edge          |           |
| add_se3_prior_quat_edge        |           |
| add_se3_prior_vec_edge         |           |
| add_plane_edge                 |           |
| add_plane_identity_edge        |           |
| add_plane_parallel_edge        |           |
| add_plane_perpendicular_edge   |           |
| add_se3_infinite_room_edge          |           |
| add_infinite_room_xplane_edge       |           |
| add_infinite_room_yplane_edge       |           |
| remove_infinite_room_xplane_edge    |           |
| remove_infinite_room_yplane_edge    |           |
| add_se3_room_edge              |           |
| add_room_xplane_edge           |           |
| add_room_2planes_edge          |           |
| add_room_4planes_edge          |           |
| add_room_xprior_edge           |           |
| add_room_yplane_edge           |           |
| add_room_yprior_edge           |           |
| add_room_room_edge             |           |
| remove_room_room_edge          |           |
| add_room_x_infinite_room_edge       |           |
| add_room_y_infinite_room_edge       |           |
| add_x_infinite_room_x_infinite_room_edge |           |
| add_y_infinite_room_y_infinite_room_edge |           |
| remove_room_xplane_edge        |           |
| remove_room_yplane_edge        |           |
| add_robust_kernel              |           |
| optimize                       |           |
| compute_landmark_marginals     |           |
| save                           |           |
| load                           |           |
| robust_kernel_factory          |           |
| graph                          |           |
| vertex_count                   |           |
| edge_count                     |           |

## GraphVisualizer

| Current             | Change To |
| ------------------- | --------- |
| create_marker_array |           |
| nh                  |           |
| map_frame_id        |           |
| color_r             |           |
| color_g             |           |
| color_b             |           |

## KeyframeMapper

| Current                  | Change To |
| ------------------------ | --------- |
| map_keyframes            |           |
| nh                       |           |
| max_keyframes_per_update |           |
| inf_calculator           |           |

## KeyframeUpdater

| Current              | Change To |
| -------------------- | --------- |
| update               |           |
| get_accum_distance   |           |
| keyframe_delta_trans |           |
| keyframe_delta_angle |           |
| is_first             |           |
| accum_distance       |           |
| prev_keypose         |           |

## LoopDetector

| Current                  | Change To |
| ------------------------ | --------- |
| detect                   |           |
| get_distance_thresh      |           |
| find_candidates          |           |
| matching                 |           |
| distance_thresh          |           |
| fitness_score_max_range  |           |
| fitness_score_thresh     |           |
| last_edge_accum_distance |           |
| registration             |           |

## MapCloudGenerator

| Current                                  | Change To |
| ---------------------------------------- | --------- |
| detect_room_neighbours                   |           |
| factor_room_neighbours                   |           |
| room_room_measurement                    |           |
| room_x_infinite_room_measurement              |           |
| room_y_infinite_room_measurement              |           |
| x_infinite_room_x_infinite_room_measurement        |           |
| y_infinite_room_y_infinite_room_measurement        |           |
| factor_room_room_constraints             |           |
| factor_room_x_infinite_room_constraints       |           |
| factor_room_y_infinite_room_constraints       |           |
| factor_x_infinite_room_room_constraints       |           |
| factor_x_infinite_room_x_infinite_room_constraints |           |
| factor_x_infinite_room_y_infinite_room_constraints |           |
| factor_y_infinite_room_room_constraints       |           |
| factor_y_infinite_room_x_infinite_room_constraints |           |
| factor_y_infinite_room_y_infinite_room_constraints |           |

## PlaneAnalyzer

| Current                     | Change To |
| --------------------------- | --------- |
| get_segmented_planes        |           |
| init_ros                    |           |
| segmented_cloud_pub\_       |           |
| compute_clusters            |           |
| compute_cloud_normals       |           |
| shadow_filter               |           |
| rainbow_color_map           |           |
| random_color                |           |
| min*seg_points*             |           |
| min_horizontal_inliers\_    |           |
| min_vertical_inliers\_      |           |
| bool use_euclidean_filter\_ |           |
| use_shadow_filter\_         |           |
| plane_extraction_frame\_    |           |
| plane_visualization_frame\_ |           |

## PlaneMapper

| Current                     | Change To |
| --------------------------- | --------- |
| map_extracted_planes        |           |
| add_planes_to_graph         |           |
| convert_plane_to_map_frame  |           |
| sort_planes                 |           |
| factor_planes               |           |
| associate_plane             |           |
| convert_plane_points_to_map |           |
| get_plane_properties        |           |
| use_point_to_plane          |           |
| plane_information           |           |
| plane_dist_threshold        |           |
| plane_points_dist           |           |
| infinite_room_min_plane_length   |           |
| room_max_plane_length       |           |
| min_plane_points            |           |
| use_infinite_room_constraint     |           |
| use_room_constraint         |           |
| nh                          |           |
| plane_utils                 |           |

## PlaneUtils

| Current                  | Change To |
| ------------------------ | --------- |
| width_between_planes     |           |
| width_between_planes     |           |
| correct_plane_d          |           |
| correct_plane_d          |           |
| room_center              |           |
| room_center              |           |
| plane_length             |           |
| plane_length             |           |
| convert_point_to_map     |           |
| get_min_segment          |           |
| check_point_neighbours   |           |
| compute_point_difference |           |
| plane_dot_product        |           |
