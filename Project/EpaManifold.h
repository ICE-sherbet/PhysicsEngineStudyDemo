// @file EpaManifold.h
// @brief 各コリジョンの衝突深度などを求める
// @author ICE
// @date 2022/08/17
//
// @details

#pragma once
#include <array>

#include "ClipUtilities.h"
#include "Vector.h"
#include "VectorUtilities.h"

namespace base_engine
{
    class CollisionComponent;
}

namespace base_engine::physics {

constexpr size_t kMaxManifoldPoints = 2;
using CollisionPoints = std::array<Vector2, kMaxManifoldPoints>;
struct Collision {
  bool isColliding = false;
  CollisionComponent* bodyA = nullptr;
  CollisionComponent* bodyB = nullptr;
  std::vector<PointPair> contactList;
  Vector2 normal;
  float depth = 0;
};

/**
 * \brief 当たり仮判定情報
 */
struct EpaManifold {
  /**
   * \brief 接触点
   */
  CollisionPoints points = CollisionPoints();
  /**
   * \brief 接触点1から2への法線
   */
  Vector2 normal = {0, 0};
  /**
   * \brief 距離
   */
  Floating depth = 0;
  bool has_collision = false;

  explicit EpaManifold() = default;

  explicit EpaManifold(InVector2 a, InVector2 b)
      : points({a, b}), has_collision(true) {
    Vector2 ba = a - b;
    depth = VectorUtilities::Length(ba);
    if (depth > 0.00001f) {
      normal = Vector2(ba / depth);
    } else {
      normal = Vector2(0, 1);
      depth = 1;
    }
  }
  explicit EpaManifold(InVector2 a, InVector2 b, InVector2 n, const float d)
      : points({a, b}), normal(n), depth(d), has_collision(true) {}
};

}  // namespace base_engine::physics
