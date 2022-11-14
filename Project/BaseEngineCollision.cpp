#include "BaseEngineCollision.h"

#include "CollisionComponent.h"
#include "IShape.h"
#include "PhysicsBodyComponent.h"

namespace base_engine {
BaseEngineCollision::~BaseEngineCollision() = default;

void BaseEngineCollision::Collide() {
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
      if (const auto body_b = body_list_[body_b_index];
          body_a->IsMatch(body_b)) {
        if (const auto manifold = body_a->Collision(body_b);
            manifold.has_collision) {
          body_a->CollisionSender(SendManifold{body_a, body_b, manifold});
          body_b->CollisionSender(SendManifold{body_b, body_a, manifold});
        }
      }
    }
  }
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
}  // namespace base_engine