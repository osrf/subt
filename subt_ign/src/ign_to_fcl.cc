#include "ign_to_fcl.hh"

#include <fcl/common/types.h>

#include <fcl/math/triangle.h>
#include <fcl/math/geometry.h>

#include <ignition/common/Mesh.hh>
#include <ignition/common/SubMesh.hh>

#include <memory>
#include <iostream>
#include <vector>

namespace subt
{

std::shared_ptr<Model> 
convert_to_fcl(const ignition::common::Mesh &_mesh)
{
  auto ret = std::make_shared<Model>();

  ret->beginModel();

  for (auto ii = 0u; ii < mesh.SubMeshCount(); ++ii)
  {
    auto submesh = mesh.SubMeshByIndex(ii).lock();
    std::vector<fcl::Vector3f> vertices;
    std::vector<fcl::Triangle> triangles;

    vertices.reserve(submesh->VertexCount());
    triangles.reserve(submesh->IndexCount() / 3);

    for(size_t jj = 0; jj < submesh->VertexCount(); ++jj)
    {
      vertices.push_back(fcl::Vector3f(
            submesh->Vertex(jj).X(),
            submesh->Vertex(jj).Y(),
            submesh->Vertex(jj).Z()));
    }

    for(size_t jj = 0; jj < submesh->IndexCount() / 3; jj += 3)
    {
      triangles.push_back(fcl::Triangle(
            submesh->Index(jj),
            submesh->Index(jj),
            submesh->Index(jj)));
    }

    ret->addSubModel(vertices, triangles);
  }

  ret->endModel();

  return ret;
}

std::shared_ptr<fcl::CollisionObjectf>
convert_to_fcl(const ignition::common::Mesh &_mesh,
               const ignition::math::Pose3d &_pose)
{
  auto model = convert_to_fcl(mesh);

  Eigen::Matrix3f rot;
  for(size_t ii = 0; ii < 3; ++ii)
  {
    for(size_t jj = 0; jj < 3; ++jj)
    {
      rot(ii, jj) = ignition::math::Matrix3d(pose.Rot())(ii, jj);
    }
  }

  auto obj = std::make_shared<fcl::CollisionObjectf>(model, rot, 
      Eigen::Vector3f(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()));
  return obj;
}

}  // namespace subt
