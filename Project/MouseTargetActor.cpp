#include "MouseTargetActor.h"

#include "Circle.h"
#include "PhysicsBodyComponent.h"
#include "ShapeRenderComponent.h"
using namespace base_engine;
void MouseTargetActor::Start() {
  shapes_.emplace_back(std::make_shared<base_engine::Rect>(-50, -50, 50, 50));
  shapes_.emplace_back(std::make_shared<base_engine::Circle>(0, 0, 50));
  auto shape = shapes_[0];
  {
    this->SetPosition({0, 0});
    shape_render_ = new ShapeRenderComponent(this, 100);
    shape_render_->SetShape(shape);
    shape_render_->SetFillMode(base_engine::FillMode::No)
        .SetColor(MOF_COLOR_YELLOW);
    collision_ = new CollisionComponent(this);
    collision_->SetShape(shape);
    body_ = new PhysicsBodyComponent(this);
    body_->SetType(physics::BodyMotionType::kDynamic);
    body_->SetGravityScale(0.0f);
  }
}

void MouseTargetActor::Input()
{
//    this->SetPosition(input_->MousePosition());
  body_->SetForce(input_->MousePosition() - GetPosition());
  return;
    if (input_->JumpFire() )
    {
      shape_type++;
      if (shape_type == shapes_.size()) shape_type = 0;
      const auto shape = shapes_[shape_type];
      shape_render_->SetShape(shape);
      collision_->SetShape(shape);

    }


}
