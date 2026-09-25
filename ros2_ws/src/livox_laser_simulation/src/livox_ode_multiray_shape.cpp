#include "livox_laser_simulation/livox_ode_multiray_shape.hpp"

#include <gazebo/common/Assert.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/ode/ODECollision.hh>
#include <gazebo/physics/ode/ODELink.hh>
#include <gazebo/physics/ode/ODEPhysics.hh>
#include <gazebo/physics/ode/ODERayShape.hh>
#include <gazebo/physics/ode/ODETypes.hh>

namespace gazebo
{
namespace physics
{
LivoxOdeMultiRayShape::LivoxOdeMultiRayShape(CollisionPtr parent)
: MultiRayShape(parent)
{
  SetName("Livox ODE multiray shape");

  super_space_id_ = dSimpleSpaceCreate(nullptr);
  ray_space_id_ = dSimpleSpaceCreate(super_space_id_);
  dGeomSetCategoryBits(reinterpret_cast<dGeomID>(ray_space_id_), GZ_SENSOR_COLLIDE);
  dGeomSetCollideBits(
    reinterpret_cast<dGeomID>(ray_space_id_), ~GZ_SENSOR_COLLIDE);

  auto link = boost::static_pointer_cast<ODELink>(collisionParent->GetLink());
  link->SetSpaceId(ray_space_id_);
  boost::static_pointer_cast<ODECollision>(collisionParent)->SetSpaceId(ray_space_id_);
}

LivoxOdeMultiRayShape::~LivoxOdeMultiRayShape()
{
  if (ray_space_id_) {
    dSpaceSetCleanup(ray_space_id_, 0);
    dSpaceDestroy(ray_space_id_);
  }
  if (super_space_id_) {
    dSpaceSetCleanup(super_space_id_, 0);
    dSpaceDestroy(super_space_id_);
  }
}

void LivoxOdeMultiRayShape::UpdateRays()
{
  auto ode = boost::dynamic_pointer_cast<ODEPhysics>(GetWorld()->Physics());
  if (!ode) {
    gzthrow("Livox MID-360 simulation requires the ODE physics engine");
  }

  boost::recursive_mutex::scoped_lock lock(*ode->GetPhysicsUpdateMutex());
  dSpaceCollide2(
    reinterpret_cast<dGeomID>(super_space_id_),
    reinterpret_cast<dGeomID>(ode->GetSpaceId()), this, &UpdateCallback);
}

void LivoxOdeMultiRayShape::UpdateCallback(
  void * data, dGeomID object1, dGeomID object2)
{
  auto * self = static_cast<LivoxOdeMultiRayShape *>(data);

  if (dGeomIsSpace(object1) || dGeomIsSpace(object2)) {
    if (
      dGeomGetSpace(object1) == self->super_space_id_ ||
      dGeomGetSpace(object2) == self->super_space_id_ ||
      dGeomGetSpace(object1) == self->ray_space_id_ ||
      dGeomGetSpace(object2) == self->ray_space_id_)
    {
      dSpaceCollide2(object1, object2, self, &UpdateCallback);
    }
    return;
  }

  auto collision_from_geom = [](dGeomID object) -> ODECollision * {
      if (dGeomGetClass(object) == dGeomTransformClass) {
        return static_cast<ODECollision *>(
          dGeomGetData(dGeomTransformGetGeom(object)));
      }
      return static_cast<ODECollision *>(dGeomGetData(object));
    };

  ODECollision * collision1 = collision_from_geom(object1);
  ODECollision * collision2 = collision_from_geom(object2);
  GZ_ASSERT(collision1, "collision1 is null");
  GZ_ASSERT(collision2, "collision2 is null");

  ODECollision * ray_collision = nullptr;
  ODECollision * hit_collision = nullptr;
  if (dGeomGetClass(object1) == dRayClass) {
    ray_collision = collision1;
    hit_collision = collision2;
    dGeomRaySetParams(object1, 0, 0);
    dGeomRaySetClosestHit(object1, 1);
  } else if (dGeomGetClass(object2) == dRayClass) {
    ray_collision = collision2;
    hit_collision = collision1;
    dGeomRaySetParams(object2, 0, 0);
    dGeomRaySetClosestHit(object2, 1);
  }

  if (!ray_collision || !hit_collision) {
    return;
  }

  dContactGeom contact;
  const int contacts = dCollide(object1, object2, 1, &contact, sizeof(contact));
  if (contacts <= 0) {
    return;
  }

  auto shape = boost::static_pointer_cast<RayShape>(ray_collision->GetShape());
  if (contact.depth < shape->GetLength()) {
    shape->SetLength(contact.depth);
    shape->SetRetro(hit_collision->GetLaserRetro());
  }
}

void LivoxOdeMultiRayShape::AddRay(
  const ignition::math::Vector3d & start,
  const ignition::math::Vector3d & end)
{
  MultiRayShape::AddRay(start, end);

  ODECollisionPtr collision(new ODECollision(collisionParent->GetLink()));
  collision->SetName("livox_ode_ray_collision");
  collision->SetSpaceId(ray_space_id_);

  ODERayShapePtr ray(new ODERayShape(collision));
  collision->SetShape(ray);
  ray->SetPoints(start, end);
  rays.push_back(ray);
}

void LivoxOdeMultiRayShape::Init()
{
  rayElem = sdf->GetElement("ray");
  scanElem = rayElem->GetElement("scan");
  horzElem = scanElem->GetElement("horizontal");
  rangeElem = rayElem->GetElement("range");
  if (scanElem->HasElement("vertical")) {
    vertElem = scanElem->GetElement("vertical");
  }
}
}  // namespace physics
}  // namespace gazebo
