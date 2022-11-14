// @file PhysicsBodyComponent.h
// @brief 物理挙動を付加
// @author ICE
// @date 2022/08/19
//
// @details

#pragma once
#include <Collision/Rectangle.h>

#include "Actor.h"
#include "CollisionComponent.h"
#include "Component.h"
#include "ComponentParameter.h"
#include "Vector.h"

namespace base_engine {
namespace physics {

/**
 * \brief 動作種別
 */
enum class BodyMotionType {
  // 物理演算の影響を受ける
  kDynamic,
  // 物理影響を受けない
  kKinematic,
  // 一切動かない
  kStatic
};
}  // namespace physics

class PhysicsBodyComponent final : public Component {
  physics::BodyMotionType motion_type_ = physics::BodyMotionType::kDynamic;
  Vector2 liner_velocity_;

 public:
  PhysicsBodyComponent(Actor* owner,
                       int update_order = kPhysicsBodyUpdateOrder);
  void Start() override;
  void OnCollision(const SendManifold& manifold) override;
  void AddForce(InVector2 force) {
    if (std::abs(liner_velocity_.y + force.y) < 30)
      liner_velocity_.y += force.y;
    if (std::abs(liner_velocity_.x + force.x) < 30)
      liner_velocity_.x += force.x;
  }
  void SetForce(InVector2 force) { liner_velocity_ = force; }
  void SetForceX(const Floating force) { liner_velocity_.x = force; }
  void SetForceY(const Floating force) { liner_velocity_.y = force; }
  Vector2 GetForce() { return liner_velocity_; }
  void Update() override {
    if (motion_type_!=physics::BodyMotionType::kDynamic)return;
    //AddForce({0, 6});

  }
  void SetType(physics::BodyMotionType type) { motion_type_ = type; }

 private:
  void Solver(const physics::Manifold& manifold,
              const PhysicsBodyComponent* target_body) const;
};
}  // namespace base_engine