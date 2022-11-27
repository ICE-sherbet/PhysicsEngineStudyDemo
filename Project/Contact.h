// @Contact.h
// @brief
// @author ICE
// @date 2022/11/27
//
// @details

#pragma once
#include "CollisionComponent.h"
#include "PhysicsBodyComponent.h"
#include "SendManifold.h"
#include "Vector.h"

namespace base_engine {
struct SendManifold;

struct CollisionComponent;

union FeaturePair {
  struct Edges {
    char inEdge1;
    char outEdge1;
    char inEdge2;
    char outEdge2;
  } e;
  int value;
};

struct Contact {
  Contact() : Pn(0.0f), Pt(0.0f), Pnb(0.0f) {}

  Vector2 position;
  Vector2 normal;
  Vector2 r1, r2;
  float separation;
  float Pn;   // accumulated normal impulse
  float Pt;   // accumulated tangent impulse
  float Pnb;  // accumulated normal impulse for position bias
  float massNormal = 0, massTangent = 0;
  float bias;
  FeaturePair feature;
};

struct ArbiterKey {
  ArbiterKey() = default;

  ArbiterKey(CollisionComponent* b1, CollisionComponent* b2) {
    if (b1 < b2) {
      body1 = b1;
      body2 = b2;
    } else {
      body1 = b2;
      body2 = b1;
    }
  }

  CollisionComponent* body1;
  CollisionComponent* body2;
};
struct VelocityConstraintPoint {
  Vector2 ra;
  Vector2 rb;
  Vector2 va;
  Vector2 vb;
  Vector2 normal;
  Vector2 tangent;
  Vector2 velocityBias;
  float bias = 0;
  float penetration = 0.0f;
  float restitution = 0.8f;
  float effectiveMassNormal = 0;
  float effectiveMassTangent = 0;
  float accumulatedNormalImpulse = 0;
  float accumulatedTangentImpulse = 0;
};

struct ContactConstraintPoint {
  ContactConstraintPoint() = default;
  ArbiterKey relation;
  float friction = 0.2f;
  bool active = true;
  Vector2 localA;
  Vector2 localB;
  CollisionComponent* bodyA = nullptr;
  CollisionComponent* bodyB = nullptr;
  VelocityConstraintPoint vcp;
};
struct ContactMaintainer {};
struct Arbiter {
  enum { MAX_POINTS = 2 };

  Arbiter() {}

  explicit Arbiter(const physics::Collision& manifold) {
    if (manifold.bodyA < manifold.bodyB) {
      body1 = manifold.bodyA;
      body2 = manifold.bodyB;
    } else {
      body1 = manifold.bodyB;
      body2 = manifold.bodyA;
    }
  }
  void Initilize(const physics::Collision& manifold) {
    int i = 0;
    for (const auto& elem : manifold.contactList) {
      bool existed = false;
      Vector2 localA = elem.pointA - body1->GetActor()->GetPosition();
      Vector2 localB = elem.pointB - body2->GetActor()->GetPosition();
      constexpr float epsilon = 0.1f;
      const bool isPointA = abs(localA.x - contacts[i].localA.x) < epsilon &&
                            abs(localA.y - contacts[i].localA.y) < epsilon;
      const bool isPointB = abs(localB.x - contacts[i].localB.x) < epsilon &&
                            abs(localB.y - contacts[i].localB.y) < epsilon;

      if (isPointA && isPointB) {
        // satisfy the condition, transmit the old accumulated value to new
        // value
        contacts[i].localA = localA;
        contacts[i].localB = localB;
        prepare(contacts[i], elem, manifold);
        existed = true;

        break;
      }

      if (existed) continue;
      // no eligible contact, push new contact points
      ContactConstraintPoint ccp;
      ccp.localA = localA;
      ccp.localB = localB;
      ccp.relation = {body1, body2};
      prepare(ccp, elem, manifold);
      contacts[numContacts] = ccp;
      ++numContacts;
    }
  }
  void Update(ContactConstraintPoint* contacts, int numContacts) {
    for (int i = 0; i < numContacts; ++i) {
      this->contacts[i] = contacts[i];
    }
  }
  void prepare(ContactConstraintPoint& ccp, const PointPair& pair,
               const physics::Collision& collision) {
    ccp.bodyA = collision.bodyA;
    ccp.bodyB = collision.bodyB;
    ccp.active = true;
    auto physicsA = collision.bodyA->GetPhysicsBody();
    auto physicsB = collision.bodyB->GetPhysicsBody();
    auto frictionA = physicsA ? physicsA->friction_ : 0;
    auto frictionB = physicsB ? physicsB->friction_ : 0;

    ccp.friction = sqrt(frictionA * frictionB);

    VelocityConstraintPoint& vcp = ccp.vcp;
    vcp.ra = pair.pointA - collision.bodyA->GetActor()->GetPosition();
    vcp.rb = pair.pointB - collision.bodyB->GetActor()->GetPosition();

    vcp.normal = collision.normal;
    vcp.tangent = VectorUtilities::Right(vcp.normal);

    const float im_a = physicsA ? physicsA->inv_mass_ : 0.001f;
    const float im_b = physicsB ? physicsB->inv_mass_ : 0.001f;
    const float ii_a = physicsA ? physicsA->inv_i_ : 0.001f;
    const float ii_b = physicsB ? physicsB->inv_i_ : 0.001f;

    const float rn_a = VectorUtilities::Cross(vcp.ra, vcp.normal);
    const float rn_b = VectorUtilities::Cross(vcp.rb, vcp.normal);

    const float rt_a = VectorUtilities::Cross(vcp.ra, vcp.tangent);
    const float rt_b = VectorUtilities::Cross(vcp.rb, vcp.tangent);

    const float kNormal = im_a + ii_a * rn_a * rn_a + im_b + ii_b * rn_b * rn_b;

    const float kTangent =
        im_a + ii_a * rt_a * rt_a + im_b + ii_b * rt_b * rt_b;

    vcp.effectiveMassNormal = kNormal == 0.0f ? 0 : 1.0f / kNormal;
    vcp.effectiveMassTangent = kTangent == 0.0f ? 0 : 1.0f / kTangent;

    vcp.restitution = 0.2f;
    vcp.penetration = collision.depth;

    Vector2 wa = VectorUtilities::Right(vcp.ra) *
                 (physicsA ? physicsA->angular_velocity_ : 0);
    Vector2 wb = VectorUtilities::Right(vcp.rb) *
                 (physicsB ? physicsB->angular_velocity_ : 0);
    vcp.va = (physicsA ? physicsA->liner_velocity_ : Vector2{0, 0}) + wa;
    vcp.vb = (physicsB ? physicsB->liner_velocity_ : Vector2{0, 0}) + wb;

    vcp.velocityBias = (vcp.va - vcp.vb) * -vcp.restitution;

    // accumulate inherited impulse
    Vector2 impulse = vcp.normal * vcp.accumulatedNormalImpulse +
                      vcp.tangent * vcp.accumulatedTangentImpulse;

    // Vector2 impulse;
    if (physicsA) {
      physicsA->applyImpulse(-impulse, vcp.ra);
    }
    if (physicsB) {
      physicsB->applyImpulse(impulse, vcp.rb);
    }
  }

  void ApplyImpulse() {
    PhysicsBodyComponent* b1 = body1->GetPhysicsBody();
    PhysicsBodyComponent* b2 = body2->GetPhysicsBody();

    for (int i = 0; i < numContacts; ++i) {
      auto c = contacts + i;
      c->vcp.ra = c->localA - body1->GetActor()->GetPosition();
      c->vcp.rb = c->localB - body2->GetActor()->GetPosition();

      // Relative velocity at contact
      Vector2 dv = b2->liner_velocity_ +
                   VectorUtilities::Right(c->vcp.rb) * b2->angular_velocity_ -
                   b1->liner_velocity_ -
                   VectorUtilities::Right(c->vcp.ra) * b1->angular_velocity_;

      // Compute normal impulse
      float vn = VectorUtilities::Dot(dv, c->vcp.normal);

      float dPn = c->vcp.effectiveMassNormal * (-vn + c->vcp.bias);

      // Apply contact impulse
      Vector2 Pn = c->vcp.normal * dPn * 0.002f;

      b1->liner_velocity_ -= Pn * b1->inv_mass_;
      //      b1->angularVelocity -= b1->invI * Cross(c->r1, Pn);

      //b2->liner_velocity_ += Pn * b2->inv_mass_;
      //      b2->angularVelocity += b2->invI * Cross(c->r2, Pn);
    }
  }
  ContactConstraintPoint contacts[MAX_POINTS];
  int numContacts = 0;

  CollisionComponent* body1;
  CollisionComponent* body2;

  // Combined friction
  float friction;
};
// This is used by std::set
inline bool operator<(const ArbiterKey& a1, const ArbiterKey& a2) {
  if (a1.body1 < a2.body1) return true;

  if (a1.body1 == a2.body1 && a1.body2 < a2.body2) return true;

  return false;
}

using ContactElement = std::pair<ArbiterKey, Arbiter>;

}  // namespace base_engine