#include "BaseEngineCollision.h"

#include <ranges>

#include "ClipUtilities.h"
#include "CollisionComponent.h"
#include "IShape.h"
#include "PhysicsBodyComponent.h"

namespace base_engine {
BaseEngineCollision::~BaseEngineCollision() = default;

physics::Collision BaseEngineCollision::Detect(
    CollisionComponent* const body_a, CollisionComponent* const body_b,
    const physics::EpaManifold& manifold) {
  physics::Collision result;

  result.normal = manifold.normal;
  result.depth = manifold.depth;
  result.isColliding = manifold.has_collision;
  result.bodyA = body_a;
  result.bodyB = body_b;

  std::vector<Vector2> vertices_a = ClipUtilities::GetVertices(body_a);
  std::vector<Vector2> vertices_b = ClipUtilities::GetVertices(body_b);
  if (g_pInput->IsKeyHold(MOFKEY_A)) {
    int n = 3;
  }
  ClipEdge edgeA =
      ClipUtilities::GetClipEdge(*body_a, vertices_a, manifold.normal);
  ClipEdge edgeB =
      ClipUtilities::GetClipEdge(*body_b, vertices_b, -manifold.normal);
  auto pair = ClipUtilities::Clip(edgeA, edgeB, manifold.normal);
  if (!pair.empty()) {
    int n = 3;
  }

  result.contactList = pair;
  return result;
}

void BaseEngineCollision::Collide() {
  // physics::PhysicsIsland island(body_list_.size());
  size_t body_size = body_list_.size();
  for (auto&& body_ : body_list_) {
    if (auto&& physics = body_->GetPhysicsBody()) {
      const auto actor = body_->GetActor();
      actor->Translation(physics->GetForce());
    }
  }
  for (int body_a_index = 0; body_a_index < body_size; ++body_a_index) {
    const auto body_a = body_list_[body_a_index];
    for (int body_b_index = body_a_index + 1; body_b_index < body_size;
         ++body_b_index) {
      const auto body_b = body_list_[body_b_index];
      ArbiterKey key(body_a, body_b);
      if (body_a->IsMatch(body_b)) {
        if (const auto manifold = body_a->Collision(body_b);
            manifold.has_collision) {
          {
            const auto collision = Detect(body_a, body_b, manifold);

            Arbiter newArb(collision);
            newArb.Initilize(collision);
            if (!arbiters.contains(key)) {
              arbiters.emplace(ContactElement(key, newArb));
            } else {
              arbiters[key] = newArb;
            }
          }
          body_a->CollisionSender(SendManifold{body_a, body_b, manifold});
          body_b->CollisionSender(SendManifold{body_b, body_a, manifold});
        } else {
          if (arbiters.contains(key)) {
            arbiters.erase(key);
          }
        }
      }
    }
  }
  Step(0.017f);
}

void BaseEngineCollision::Register(CollisionComponent* component) {
  body_list_.emplace_back(component);
}

void BaseEngineCollision::Remove(CollisionComponent* component) {
  body_list_.erase(remove(body_list_.begin(), body_list_.end(), component),
                   body_list_.end());
}

void BaseEngineCollision::SendComponentsMessage(Component* component,
                                                const SendManifold& manifold) {
  component->OnCollision(manifold);
}

void BaseEngineCollision::Step(float dt) {
  
  {

    for (int i = 0; i < 10; ++i) {
      SolveVelocity(dt);
    }
    for (int i = 0; i < 5; ++i)
    {
      SolvePosition(dt);
        
    }
  }

  for (auto&& collision : body_list_) {
      const auto body = collision->GetPhysicsBody();
    if (!body) continue;
    if (body->inv_mass_ == 0.0f) continue;

    body->liner_velocity_ += (body->force_ * body->inv_mass_) * 0.017f;
    body->angular_velocity_ += 0.017f * body->inv_i_ * body->torque_;
    if (body->GetType() != physics::BodyMotionType::kStatic) {
      body->liner_velocity_ += gravity_ * body->gravity_scale_;
    }
  }
  for (auto&& collision : body_list_) {
    auto b = collision->GetPhysicsBody();
    b->Step(dt);
  }
}
}  // namespace base_engine