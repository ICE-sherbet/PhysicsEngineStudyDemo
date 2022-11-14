#include "PhysicsGameSetup.h"

#include "ClickCollisionCreatorComponent.h"
#include "InputManager.h"
#include "MouseTargetActor.h"

void PhysicsGameSetup::SetUp() {
  auto inputActor = new base_engine::InputActor(game_);
  auto input = new InputManager(inputActor);

  auto mouse_follow_actor_ = new MouseTargetActor(game_);
  mouse_follow_actor_->SetInput(input);
  auto PhysicsCreatorActor = new base_engine::Actor(game_);
  auto creater = new ClickCollisionCreatorComponent(PhysicsCreatorActor);
  creater->SetInput(input);
}
