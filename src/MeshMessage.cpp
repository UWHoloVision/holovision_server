#include "MeshMessage.h"

namespace holovision {

MeshMessage create_mesh_message(pcl::PolygonMesh::Ptr mesh) {
  MeshMessage msg;
  // need to convert pointcloud2 to points
  pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh->cloud, *mesh_cloud);
  msg.n_points = mesh_cloud->points.size(); // number of points
  msg.n_triangles = mesh->polygons.size(); // number of triangles
  msg.points.reserve(msg.n_points * 3); // 3 values (x, y, z) per point
  msg.triangles.reserve(msg.n_triangles * 3); // 3 points per triangle
  // copy points
  for (auto i = 0; i < msg.n_points; i++) {
    pcl::PointXYZ& point = mesh_cloud->at(i);
    msg.points.push_back(point.x);
    msg.points.push_back(point.y);
    msg.points.push_back(-1*point.z); // unity coordinates rhs -> lhs
  }
  // copy triangle indices
  for (auto& polygon: mesh->polygons) {
    assert(polygon.vertices.size() == 3);
    // triangle indices
    msg.triangles.insert(
      msg.triangles.end(), 
      polygon.vertices.begin(), 
      polygon.vertices.end()
    );
  }
  assert(msg.points.size() == msg.n_points * 3);
  assert(msg.triangles.size() == msg.n_triangles * 3);
  return msg;
}

void write_mesh_message(MeshMessage&& msg, std::ostream& os) {
  int32_t n_points = msg.n_points;
  std::cout << "NPoints: " << n_points << std::endl;
  assert(os.write(
    reinterpret_cast<char*>(&n_points), 
    sizeof(n_points)
  ));
  int32_t n_triangles = msg.n_triangles;
  std::cout << "NTriangles: " << n_triangles << std::endl;
  assert(os.write(
    reinterpret_cast<char*>(&n_triangles), 
    sizeof(n_triangles)
  ));
  // copy all flattened points
  for(auto point: msg.points) {
    assert(os.write(
      reinterpret_cast<char*>(&point),
      sizeof(point)
    ));
  }
  // copy all triangle indices
  for(auto triangle: msg.triangles) {
    assert(os.write(
      reinterpret_cast<char*>(&triangle),
      sizeof(triangle)
    ));
  }
}


} // namespace holovision
