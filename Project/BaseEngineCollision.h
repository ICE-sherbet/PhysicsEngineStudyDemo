#pragma once
#include <Mof.h>

#include <algorithm>
#include <map>
#include <ranges>

#include "Contact.h"
#include "IBaseEngineCollider.h"
#include "Vector.h"

namespace base_engine {

class BaseEngineCollision final : public IBaseEngineCollider {
 public:
  ~BaseEngineCollision() override;
  void Collide() override;
  void Register(CollisionComponent* component) override;
  void Remove(CollisionComponent* component) override;

  void SendComponentsMessage(Component* component,
                             const SendManifold& manifold) override;

 private:
  void Step(float dt);

  void SolveVelocity(float dt) {
    for (auto& val : arbiters | std::views::values) {
      if (val.numContacts == 0 || !val.contacts[0].active) continue;

      for (auto&& ccp : val.contacts) {
        auto& vcp = ccp.vcp;
        auto physics_a = ccp.bodyA->GetPhysicsBody();
        auto physics_b = ccp.bodyB->GetPhysicsBody();
        auto av_a = physics_a ? physics_a->angular_velocity_ : 0;
        auto av_b = physics_b ? physics_b->angular_velocity_ : 0;
        auto v_a = physics_a ? physics_a->liner_velocity_ : Vector2{0, 0};
        auto v_b = physics_b ? physics_b->liner_velocity_ : Vector2{0, 0};
        Vector2 wa = VectorUtilities::Right(vcp.ra) * av_a;
        Vector2 wb = VectorUtilities::Right(vcp.rb) * av_b;
        vcp.va = v_a + wa;
        vcp.vb = v_b + wb;

        Vector2 dv = vcp.va - vcp.vb;
        float jv = -1.0f * VectorUtilities::Dot(-vcp.normal, (dv));
        float lambda_n = vcp.effectiveMassNormal * jv;
        float oldImpulse = vcp.accumulatedNormalImpulse;
        vcp.accumulatedNormalImpulse = std::max(oldImpulse + lambda_n, 0.0f);
        lambda_n = vcp.accumulatedNormalImpulse - oldImpulse;

        Vector2 impulse_n = vcp.normal * lambda_n * ccp.friction;
        if (impulse_n.y < -0.1) {
          int n = 3;
        }
        if (physics_a) physics_a->applyImpulse(-impulse_n, vcp.ra);
        if (physics_b) physics_b->applyImpulse(impulse_n, vcp.rb);

        vcp.va = v_a + VectorUtilities::Right(vcp.ra) * av_a;
        vcp.vb = v_b + VectorUtilities::Right(vcp.rb) * av_b;

        dv = vcp.va - vcp.vb;

        float jvt = VectorUtilities::Dot(vcp.tangent, dv);
        float lambda_t = vcp.effectiveMassTangent * -jvt;

        float maxT = ccp.friction * vcp.accumulatedNormalImpulse;
        oldImpulse = vcp.accumulatedTangentImpulse;
        vcp.accumulatedTangentImpulse =
            std::clamp(oldImpulse + lambda_t, -maxT, maxT);

        lambda_t = vcp.accumulatedTangentImpulse - oldImpulse;

        Vector2 impulse_t = vcp.tangent * lambda_t;

        //if (physics_a) physics_a->applyImpulse(-impulse_t, vcp.ra);
        //if (physics_b) physics_b->applyImpulse(impulse_t, vcp.rb);
      }
    }
  }
  void SolvePosition(float dt) {
    if (g_pInput->IsKeyHold(MOFKEY_A)) {
      int n = 3;
    }
    for (auto& elem : arbiters | std::views::values) {
      if (elem.numContacts == 0 || !elem.contacts[0].active) continue;
      for (auto&& ccp : elem.contacts) {
        auto&& vcp = ccp.vcp;
        const auto bodyA = ccp.bodyA;
        const auto bodyB = ccp.bodyB;
        Vector2 pa = vcp.ra + bodyA->GetActor()->GetPosition();
        Vector2 pb = vcp.rb + bodyB->GetActor()->GetPosition();
        Vector2 c = pb - pa;

        float bias = std::max(VectorUtilities::Length(c) - 0.02f, 0.0f) * 0.03f;
        float lambda = vcp.effectiveMassNormal * bias;

        Vector2 impulse = vcp.normal * lambda * vcp.penetration;

        const auto physics_a = ccp.bodyA->GetPhysicsBody();
        const auto physics_b = ccp.bodyB->GetPhysicsBody();
        const auto type_a =
            physics_a ? physics_a->GetType() : physics::BodyMotionType::kStatic;
        const auto type_b =
            physics_a ? physics_b->GetType() : physics::BodyMotionType::kStatic;
        if (type_a != physics::BodyMotionType::kStatic) {
          bodyA->GetActor()->SetPosition(bodyA->GetActor()->GetPosition() -
                                         impulse * physics_a->inv_mass_);
          physics_a->rotation_ +=
              VectorUtilities::Cross(vcp.ra, impulse) * physics_a->inv_i_;
        }
        if (type_b != physics::BodyMotionType::kStatic) {
          bodyB->GetActor()->SetPosition(bodyB->GetActor()->GetPosition() +
                                         (impulse * physics_b->inv_mass_));

          physics_b->rotation_ -=
              VectorUtilities::Cross(vcp.rb, impulse) * physics_b->inv_i_;
        }
      }
    }
  }
  static physics::Collision Detect(CollisionComponent* const body_a,
                                   CollisionComponent* const body_b,
                                   const physics::EpaManifold& manifold);
  Vector2 gravity_{0, 14};
  std::map<ArbiterKey, Arbiter> arbiters;
};
}  // namespace base_engine
