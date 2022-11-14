#include "ClickCollisionCreatorComponent.h"

#include "CollisionComponent.h"
#include "Mof.h"
#include "PhysicsBodyComponent.h"
#include "ShapeRenderComponent.h"

using namespace base_engine;

void ClickCollisionCreatorComponent::SetInput(InputManager* input)
{ input_ = input; }

void ClickCollisionCreatorComponent::Start() {
  shape_ = std::make_shared<base_engine::Circle>(0, 0, 10);
  ground_ = std::make_shared<base_engine::Rect>(0, 0, 1080, 120);
  auto box_ = std::make_shared<base_engine::Rect>(0, 0, 100, 100);

  {
    auto actor = new Actor(this->owner_->GetGame());
    actor->SetPosition({0, 600});
    auto shape_render = new ShapeRenderComponent(actor, 100);
    shape_render->SetShape(ground_);
    shape_render->SetFillMode(base_engine::FillMode::No)
        .SetColor(MOF_COLOR_BLUE);
    auto collision = new CollisionComponent(actor);
    collision->SetShape(ground_);
    auto physics = new PhysicsBodyComponent(actor);
    physics->SetType(physics::BodyMotionType::kStatic);
  }

  {
    auto actor = new Actor(this->owner_->GetGame());
    actor->SetPosition({100, 200});
    auto shape_render = new ShapeRenderComponent(actor, 100);
    shape_render->SetShape(box_);
    shape_render->SetFillMode(base_engine::FillMode::No)
        .SetColor(MOF_COLOR_BLUE);
    auto collision = new CollisionComponent(actor);
    collision->SetShape(box_);
    auto physics = new PhysicsBodyComponent(actor);
    physics->SetType(physics::BodyMotionType::kDynamic);
  }

      {
    auto actor = new Actor(this->owner_->GetGame());
    actor->SetPosition({500, 300});
    auto shape_render = new ShapeRenderComponent(actor, 100);
    shape_render->SetShape(box_);
    shape_render->SetFillMode(base_engine::FillMode::No)
        .SetColor(MOF_COLOR_BLUE);
    auto collision = new CollisionComponent(actor);
    collision->SetShape(box_);
    auto physics = new PhysicsBodyComponent(actor);
    physics->SetType(physics::BodyMotionType::kDynamic);
  }

}

void ClickCollisionCreatorComponent::ProcessInput() {
  if (g_pInput->IsMouseKeyPush(0)) {
    PhysicsColliderCreate();
  }
}

void ClickCollisionCreatorComponent::PhysicsColliderCreate() {
  auto actor = new Actor(this->owner_->GetGame());
  Vector2 pos;
  g_pInput->GetMousePos(pos);
  actor->SetPosition(pos);
  auto shape_render = new ShapeRenderComponent(actor, 100);
  shape_render->SetShape(shape_);
  shape_render->SetFillMode(base_engine::FillMode::Yes)
      .SetColor(MOF_COLOR_GREEN);
  auto collision = new CollisionComponent(actor);
  collision->SetShape(shape_);
  new PhysicsBodyComponent(actor);
}
