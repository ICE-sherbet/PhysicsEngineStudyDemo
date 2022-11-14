// @PhysicsIsland.h
// @brief
// @author ICE
// @date 2022/11/14
//
// @details

#pragma once
#include "PhysicsBodyComponent.h"
#include "PhysicsMath.h"
#include "PhysicsTypes.h"

namespace base_engine::physics {
struct PhysicsPosition {
  PhysicsVec2 c;
  float a;
};

struct PhysicsVelocity {
  PhysicsVec2 v;
  float w;
};
using PPosition = PhysicsPosition;
using PVelocity = PhysicsVelocity;
class PhysicsIsland {
 public:
  explicit PhysicsIsland(int32 body_capacity) {
    bodies_.resize(body_capacity);
    positions_.resize(body_capacity);
    velocities_.resize(body_capacity);
  }
  void Add(PhysicsBodyComponent* body) {
    bodies_[body_count_] = body;
    ++body_count_;
  }
  void Solve(InVector2 gravity, bool allowSleep) {
    float h = 0.017f;

    // 配列を構造体で初期化する。
    for (int32 i = 0; i < body_count_; ++i) {
      const auto b = bodies_[i];

      const PhysicsVec2 c = {b->owner_->GetPosition().x,
                             b->owner_->GetPosition().y};
      const float a = b->sweep_.a;

      Vector2 v = b->GetForce();

      if (b->GetType() == BodyMotionType::kDynamic) {
        v += (gravity * b->GetGravityScale() + b->GetForce());
      }

      positions_[i].c = c;
      positions_[i].a = a;
      velocities_[i].v = v;
    }

    for (int32 i = 0; i < 10; ++i) {
    }

    // Integrate positions
    for (int32 i = 0; i < body_count_; ++i) {
      auto c = positions_[i].c;
      auto a = positions_[i].a;
      auto v = velocities_[i].v;

      // Check for large velocities
      if (auto translation = h * v; PhysicsDot(translation, translation) > 4) {
        float ratio = 4 / translation.Length();
        v *= ratio;
      }

      // Integrate
      c += h * v;

      positions_[i].c = c;
      positions_[i].a = a;
      velocities_[i].v = v;
    }

    for (int32 i = 0; i < body_count_; ++i) {
      const auto body = bodies_[i];
      body->sweep_.c = positions_[i].c;
      body->sweep_.a = positions_[i].a;
      body->liner_velocity_.x = velocities_[i].v.x;
      body->liner_velocity_.y = velocities_[i].v.y;
      if (body->liner_velocity_.y > 6)
      {
        body->liner_velocity_.y = 6;
      }
      if (body->liner_velocity_.y < -6) {
        body->liner_velocity_.y = -6;
      }
      body->owner_->SetPosition({body->sweep_.c.x, body->sweep_.c.y});
    }
  }

 private:
  int32 body_count_ = 0;
  std::vector<PhysicsBodyComponent*> bodies_;
  std::vector<PPosition> positions_;
  std::vector<PVelocity> velocities_;
};

}  // namespace base_engine::physics
