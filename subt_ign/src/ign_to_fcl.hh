#ifndef IGN_TO_FCL_HH_
#define IGN_TO_FCL_HH_

#include <fcl/narrowphase/collision_object.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/math/bv/OBBRSS.h>

#include <ignition/common/Mesh.hh>
#include <ignition/math/Pose3.hh>

#include <subt_ign/VisibilityTypes.hh>

namespace subt
{

using Model = fcl::BVHModel<fcl::OBBRSSf>;

std::shared_ptr<Model> 
convert_to_fcl(const ignition::common::Mesh& mesh);

std::shared_ptr<fcl::CollisionObjectf>
convert_to_fcl(const ignition::common::Mesh& mesh,
               const ignition::math::Pose3d& pose);

}



#endif
