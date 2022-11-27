#include "PhysicsBodyComponent.h"

#include "CollisionComponent.h"
#include "SendManifold.h"

base_engine::PhysicsBodyComponent::PhysicsBodyComponent(Actor* owner,
                                                        int update_order)
    : Component(owner, update_order) {}

void base_engine::PhysicsBodyComponent::Start() {
  const auto collider = owner_->GetComponent<CollisionComponent>();
  if (collider.expired()) return;
  collider.lock()->SetPhysicsBody(this);
  mass_ = FLT_MAX;
  inv_mass_ = 0.0f;
  i_ = FLT_MAX;
  inv_i_ = 0.0f;
}
using enum base_engine::physics::BodyMotionType;
void base_engine::PhysicsBodyComponent::OnCollision(
    const SendManifold& manifold) {
  const auto this_collision = manifold.collision_a;
  const auto target_collision = manifold.collision_b;
  if (motion_type_ != kDynamic) return;

  if (this_collision->GetTrigger() || target_collision->GetTrigger()) return;

  const auto collision_depth = manifold.manifold;
  const auto target_body = target_collision->GetPhysicsBody();

  Solver(collision_depth, target_body);
  auto force = GetForce();

  force.x = std::abs(force.x);
  force.y = std::abs(force.y);
  AddForce(force * collision_depth.normal);
}

void base_engine::PhysicsBodyComponent::Solver(
    const physics::Manifold& manifold,
    const PhysicsBodyComponent* target_body) const {
  if (target_body == nullptr) {
    owner_->Translation({manifold.normal * -manifold.depth});
    return;
  }

  const auto target_type = target_body->motion_type_;
  if (motion_type_ == kDynamic && target_type == kDynamic) {
    owner_->Translation({manifold.normal * manifold.depth * 0.5f});
    return;
  }

  if (motion_type_ == kDynamic && target_type != kDynamic) {
    owner_->Translation({manifold.normal * manifold.depth});
    return;
  }

  if (motion_type_ == kKinematic && target_type == kKinematic) {
    owner_->Translation({manifold.normal * -manifold.depth * 0.5});
    return;
  }
}
