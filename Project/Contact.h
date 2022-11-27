// @Contact.h
// @brief
// @author ICE
// @date 2022/11/27
//
// @details

#pragma once
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
  float massNormal, massTangent;
  float bias;
  FeaturePair feature;
};

struct ArbiterKey {
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

struct Arbiter {
  enum { MAX_POINTS = 2 };

  explicit Arbiter(const SendManifold& manifold)
  {
    if (manifold.collision_a < manifold.collision_b) {
      body1 = manifold.collision_a;
      body2 = manifold.collision_b;
    } else {
      body1 = manifold.collision_b;
      body2 = manifold.collision_a;
    }
  }

  void Update(Contact* contacts, int numContacts);

  void PreStep(float inv_dt);
  void ApplyImpulse();

  Contact contacts[MAX_POINTS];
  int numContacts;

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