// @ClickCollisionCreatorComponent.h
// @brief
// @author ICE
// @date 2022/10/18
//
// @details

#pragma once
#include "Circle.h"
#include "Component.h"
#include "IShape.h"
#include "InputManager.h"

class ClickCollisionCreatorComponent : base_engine::Component {
  InputManager* input_ = nullptr;
  std::shared_ptr<base_engine::IShape> shape_;
  std::shared_ptr<base_engine::IShape> ground_;
 public:
  ClickCollisionCreatorComponent(base_engine::Actor* owner,
                                 int update_order = 100)
      : Component(owner, update_order) {}
  void SetInput(InputManager* input);;
  void Start() override;

  void Update() override {}

  void ProcessInput() override;

    void PhysicsColliderCreate();
};
