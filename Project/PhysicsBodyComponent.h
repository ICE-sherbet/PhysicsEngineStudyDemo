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
#include "PhysicsMath.h"
#include "Vector.h"

namespace base_engine {
namespace physics {
class PhysicsIsland;

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
 public:
  PhysicsBodyComponent(Actor* owner,
                       int update_order = kPhysicsBodyUpdateOrder);
  void Start() override;
  void OnCollision(const SendManifold& manifold) override;
  void AddForce(InVector2 force) {
    if (std::abs(liner_velocity_.y + force.y) < 1) {
      liner_velocity_.y += force.y;
      if (liner_velocity_.y > 1) liner_velocity_.y = 1;
      if (liner_velocity_.y < -1) liner_velocity_.y = -1;
    }
    if (std::abs(liner_velocity_.x + force.x) < 3) liner_velocity_.x += force.x;
  }
  void SetForce(InVector2 force) { liner_velocity_ = force; }
  void SetForceX(const Floating force) { liner_velocity_.x = force; }
  void SetForceY(const Floating force) { liner_velocity_.y = force; }
  Vector2 GetForce() { return liner_velocity_; }
  [[nodiscard]] physics::PhysicsSweep GetSweep() const { return sweep_; }
  [[nodiscard]] float GetGravityScale() const { return gravity_scale_; }
  [[nodiscard]] physics::BodyMotionType GetType() const { return motion_type_; }

  void Update() override {
    if (motion_type_ != physics::BodyMotionType::kDynamic) return;
    // AddForce({0, 6});
  }
  void SetType(physics::BodyMotionType type) { motion_type_ = type; }

  void SetGravityScale(float gravity_scale) { gravity_scale_ = gravity_scale; }

  float rotation_ = 0;

  Vector2 liner_velocity_ = {0, 0};
  float angular_velocity_ = 0;

  Vector2 force_ = {0, 0};
  float torque_ = 0;

  float friction_ = 0.2f;
  float mass_ = 0;
  float inv_mass_ = 0;
  float i_ = 0, inv_i_ = 0;

  float gravity_scale_ = 1.0f;
  physics::BodyMotionType motion_type_ = physics::BodyMotionType::kDynamic;


 private:
  friend class physics::PhysicsIsland;

  void Solver(const physics::Manifold& manifold,
              const PhysicsBodyComponent* target_body) const;


  physics::PhysicsSweep sweep_{};
};
}  // namespace base_engine