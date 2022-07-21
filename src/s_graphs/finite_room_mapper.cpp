// SPDX-License-Identifier: BSD-2-Clause

#include <s_graphs/room_mapper.hpp>

namespace s_graphs {

FiniteRoomMapper::FiniteRoomMapper(const ros::NodeHandle& private_nh) {
  plane_utils.reset(new PlaneUtils());
}

FiniteRoomMapper::~FiniteRoomMapper() {}

}  // namespace s_graphs