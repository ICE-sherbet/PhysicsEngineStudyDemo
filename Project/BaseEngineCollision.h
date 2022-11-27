#pragma once
#include <map>

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
  Vector2 gravity_{0, 1};
  std::map<ArbiterKey, Arbiter> arbiters;
};
}  // namespace base_engine
