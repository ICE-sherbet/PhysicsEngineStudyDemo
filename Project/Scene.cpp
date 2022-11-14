#include "Scene.h"

#include "Actor.h"

namespace base_engine::scene {

void Scene::Clear() const
{
  for (auto&& actor : actors_) {
    actor.lock()->SetState(Actor::kDead);
  }
}
}  // namespace base_engine::scene