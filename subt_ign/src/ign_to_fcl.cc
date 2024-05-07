#include "ign_to_fcl.hh"

#include <fcl/config.h>
#if FCL_MAJOR_VERSION == 0 && FCL_MINOR_VERSION == 5
#include <fcl/data_types.h>
#include <fcl/math/matrix_3f.h>
#include <fcl/math/vec_3f.h>
#else
#include <fcl/common/types.h>
#endif

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

  for (auto ii = 0u; ii < _mesh.SubMeshCount(); ++ii)
  {
    auto submesh = _mesh.SubMeshByIndex(ii).lock();
#if FCL_MAJOR_VERSION == 0 && FCL_MINOR_VERSION == 5
    std::vector<fcl::Vec3f> vertices;
#else
    std::vector<fcl::Vector3d> vertices;
#endif
    std::vector<fcl::Triangle> triangles;

    vertices.reserve(submesh->VertexCount());
    triangles.reserve(submesh->IndexCount() / 3);

    for(size_t jj = 0; jj < submesh->VertexCount(); ++jj)
    {
#if FCL_MAJOR_VERSION == 0 && FCL_MINOR_VERSION == 5
      vertices.push_back(fcl::Vec3f(
#else
      vertices.push_back(fcl::Vector3d(
#endif
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

#if FCL_MAJOR_VERSION == 0 && FCL_MINOR_VERSION == 5
std::shared_ptr<fcl::CollisionObject>
#else
std::shared_ptr<fcl::CollisionObjectd>
#endif
convert_to_fcl(const ignition::common::Mesh &_mesh,
               const ignition::math::Pose3d &_pose)
{
  auto model = convert_to_fcl(_mesh);

#if FCL_MAJOR_VERSION == 0 && FCL_MINOR_VERSION == 5
  fcl::Matrix3f rot;
#else
  fcl::Matrix3d rot;
#endif
  for(size_t ii = 0; ii < 3; ++ii)
  {
    for(size_t jj = 0; jj < 3; ++jj)
    {
      rot(ii, jj) = ignition::math::Matrix3d(_pose.Rot())(ii, jj);
    }
  }

#if FCL_MAJOR_VERSION == 0 && FCL_MINOR_VERSION == 5
  auto obj = std::make_shared<fcl::CollisionObject>(model, rot,
      fcl::Vec3f(_pose.Pos().X(), _pose.Pos().Y(), _pose.Pos().Z()));
#else
  auto obj = std::make_shared<fcl::CollisionObjectd>(model, rot,
      fcl::Vector3d(_pose.Pos().X(), _pose.Pos().Y(), _pose.Pos().Z()));
#endif
  return obj;
}

}  // namespace subt
