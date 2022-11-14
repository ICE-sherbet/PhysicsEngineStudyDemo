#include "Game.h"

#include <Utilities/GraphicsUtilities.h>

#include "CameraComponent.h"
#include "CollisionComponent.h"

#include "InputComponent.h"
#include "InputManager.h"
#include "PhysicsGameSetup.h"

#include "RenderComponent.h"
#include "TexturePaths.h"

base_engine::IBaseEngineCollider* b_collision;
namespace base_engine {
bool Game::Initialize() {
  game_data_.Register();
  auto setup = PhysicsGameSetup(this);
  setup.SetUp();


  b_collision = BASE_ENGINE(Collider);
  return true;
}

void Game::Update() {
  CreateObjectRegister();
  ProcessInput();
  UpdateGame();
  b_collision->Collide();
}

void Game::Shutdown() { actors_.clear(); }

void Game::AddActor(Actor* actor) { pending_actors_.emplace_back(actor); }

void Game::RemoveActor(Actor* actor) {
  if (const auto iter = std::ranges::find_if(
          actors_, [actor](const ActorPtr& n) { return n.get() == actor; });
      iter != actors_.end()) {
    std::iter_swap(iter, actors_.end() - 1);
    actors_.pop_back();
  }
}
ActorWeakPtr Game::GetActor(Actor* actor) {
  auto iter = std::ranges::find_if(
      actors_, [actor](const ActorPtr& n) { return n.get() == actor; });
  if (iter != actors_.end()) return *iter;

  iter = std::ranges::find_if(
      pending_actors_, [actor](const ActorPtr& n) { return n.get() == actor; });
  if (iter != actors_.end()) return *iter;

  return {};
}
void Game::AddSprite(RenderComponent* render_component) {
  const int my_draw_order = render_component->GetDrawOrder();
  auto iterator = sprites_.begin();
  for (; iterator != sprites_.end(); ++iterator) {
    if (my_draw_order < (*iterator)->GetDrawOrder()) {
      break;
    }
  }

  sprites_.insert(iterator, render_component);
}

void Game::RemoveSprite(RenderComponent* render_component) {
  auto iterator = std::ranges::find(sprites_, render_component);
  sprites_.erase(iterator);
}

void Game::CreateObjectRegister() {
  updating_actors_ = true;
  for (auto& pending_actor : pending_actors_) {
    pending_actor->StartActor();
    actors_.emplace_back(pending_actor);
  }
  pending_actors_.clear();

  for (auto&& actor : actors_) {
    actor->AddComponent();
  }
  updating_actors_ = false;
}

void Game::ProcessInput() {
  updating_actors_ = true;
  for (const auto& actor : actors_) {
    actor->ProcessInput();
  }
  updating_actors_ = false;
}

void Game::UpdateGame() {
  updating_actors_ = true;

  for (const auto& actor : actors_) {
    actor->UpdateActor();
  }
  updating_actors_ = false;

  std::vector<ActorPtr> dead_actors;
  for (const auto& actor : actors_) {
    if (actor->GetState() == Actor::kDead) {
      dead_actors.emplace_back(actor);
    }
  }
  for (auto&& actor : dead_actors) {
    actor.reset();
  }
}

void Game::Render() const {
  for (const auto sprite : sprites_) {
    if (sprite->GetEnabled()) sprite->Draw();
  }
  for (auto& func : debug_render_) {
    func();
  }
  Mof::CGraphicsUtilities::RenderString(0, 0, MOF_COLOR_BLACK, "FPS:%d",
                                        Mof::CUtilities::GetFPS());
}

}  // namespace base_engine