#ifndef IGN_TO_FCL_HH_
#define IGN_TO_FCL_HH_

#include <fcl/collision_object.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/BV/OBBRSS.h>

#include <ignition/common/Mesh.hh>
#include <ignition/math/Pose3.hh>

#include <subt_ign/VisibilityTypes.hh>

namespace subt
{

/// \brief Convenience alias for the type of fcl model that is
/// being used.  
/// BVH = Bounding Volume Heirarchy
/// OBB = Oriented Bounding Box
/// RSS = Rectange Swept Sphere
using Model = fcl::BVHModel<fcl::OBBRSS>;

/// \brief Create an fcl model from an ignition mesh.
///
/// Note that only triangle meshes are currently supported.
///
/// \param[in] _mesh mesh to convert to fcl model.
/// \return model when successfully converted otherwise nullptr
std::shared_ptr<Model> 
convert_to_fcl(const ignition::common::Mesh &_mesh);

/// \brief Create an fcl CollisionObject from an ignition mesh+pose
///
/// Note that only triangle meshes are currently supported.
///
/// \param[in] _mesh mesh to convert to fcl model.
/// \param[in] _pose world pose of the model.
/// \return collision object when successfully converted otherwise nullptr
std::shared_ptr<fcl::CollisionObject>
convert_to_fcl(const ignition::common::Mesh &_mesh,
               const ignition::math::Pose3d &_pose);

}  // namespace subt

#endif  // IGN_TO_FCL_HH_
