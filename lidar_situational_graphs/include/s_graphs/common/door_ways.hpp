/*
Copyright (c) 2023, University of Luxembourg
All rights reserved.

Redistributions and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*/

#ifndef DOOR_WAYS_HPP
#define DOOR_WAYS_HPP

#include <g2o/types/slam3d_addons/plane3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Eigen>
#include <g2o/vertex_doorway.hpp>

namespace g2o {
class VertexDoorWay;
class HyperGraph;
class SparseOptimizer;
}  // namespace g2o

namespace s_graphs {

/**
 * @brief
 *
 * @param id, Door's I.D for graph slam
 * @param prior_id, Door's I.D from prior knowledge
 * @param room1_id, I.D of 1st room
 * @param room2_id, I.D of 2nd room
 * @param door_pos_w, Door's position in WOrld frame
 * @param door_pos_r1, Door's position in 1st room's frame
 * @param door_pos_r1, Door's position in 2nd room's frame
 */
struct DoorWays {
 public:
  int id;
  int prior_id;
  Eigen::Vector3d door_pos_w, door_pos_r1, door_pose_r2;
  int room1_id, room2_id;
  g2o::VertexDoorWay* node;  // node instance
};

}  // namespace s_graphs
#endif  // DOOR_WAYS_HPP
