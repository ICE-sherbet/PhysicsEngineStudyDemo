// @MouseTargetActor.h
// @brief
// @author かき氷氷味
// @date 2022/11/08
// 
// @details

#pragma once
#include "Actor.h"
#include "InputManager.h"
#include "PhysicsBodyComponent.h"
#include "ShapeRenderComponent.h"

class MouseTargetActor : public base_engine::Actor
{
  std::vector<std::shared_ptr<base_engine::IShape>> shapes_;
  base_engine::ShapeRenderComponent* shape_render_;
  base_engine::PhysicsBodyComponent* body_;
  base_engine::CollisionComponent* collision_;
  InputManager* input_;

  int shape_type = 0;

 public:
  explicit MouseTargetActor(base_engine::Game* game)
      : Actor(game)
  {
  }

  void SetInput(InputManager* input)
  {
      input_ = input;
  }

  void Start() override;

    void Input() override;
    
};
