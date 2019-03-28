#pragma once

#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <boost/array.hpp>
#include <boost/asio.hpp>

namespace holovision {

struct MeshMessage {
  int32_t n_points;
  int32_t n_triangles;
  // x, y, z coordinates in a flat vector
  std::vector<float> points;
  // triangle indices
  std::vector<int32_t> triangles;
};

MeshMessage create_mesh_message(pcl::PolygonMesh::Ptr);

void write_mesh_message(MeshMessage&&, std::ostream&);

} // namespace holovision
