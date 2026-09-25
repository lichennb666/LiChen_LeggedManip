#ifndef LIVOX_LASER_SIMULATION__LIVOX_ODE_MULTIRAY_SHAPE_HPP_
#define LIVOX_LASER_SIMULATION__LIVOX_ODE_MULTIRAY_SHAPE_HPP_

#include <gazebo/ode/common.h>
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/util/system.hh>
#include <ignition/math6/ignition/math.hh>

#include <vector>

namespace gazebo
{
namespace physics
{
class GZ_PHYSICS_VISIBLE LivoxOdeMultiRayShape : public MultiRayShape
{
public:
  explicit LivoxOdeMultiRayShape(CollisionPtr parent);
  ~LivoxOdeMultiRayShape() override;

  void UpdateRays() override;
  void Init() override;
  void AddRay(
    const ignition::math::Vector3d & start,
    const ignition::math::Vector3d & end);

  std::vector<RayShapePtr> & RayShapes() {return rays;}

private:
  static void UpdateCallback(void * data, dGeomID object1, dGeomID object2);

  dSpaceID super_space_id_{};
  dSpaceID ray_space_id_{};
};
}  // namespace physics
}  // namespace gazebo

#endif  // LIVOX_LASER_SIMULATION__LIVOX_ODE_MULTIRAY_SHAPE_HPP_
