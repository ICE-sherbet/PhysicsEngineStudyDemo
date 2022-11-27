// @ClipUtilities.h
// @brief
// @author ICE
// @date 2022/11/27
//
// @details

#pragma once
#include <span>
#include <vector>

#include "IShape.h"
#include "Vector.h"


#undef max
#undef min
namespace base_engine {
    class CollisionComponent;

    struct ClipEdge {
    Mof::Vector2 p1;
  Vector2 p2;
  Vector2 normal;

  [[nodiscard]] bool IsEmpty() const {
    return p1.x == 0 && p1.y == 0 && p2.x == 0 && p2.y == 0;
  }
};

struct PointPair {
  PointPair() = default;
  Vector2 pointA;
  Vector2 pointB;
  bool isEmpty() const {
    return pointA == Vector2{0, 0} && pointB == Vector2{0, 0};
  }
  bool operator==(const PointPair& other) const {
    return (other.pointA == this->pointA && other.pointB == this->pointB) ||
           (other.pointB == this->pointA && other.pointA == this->pointB);
  }
};
class ClipUtilities {
 public:
  static inline bool SameSign(const float a, const float b);
  static bool IsPointOnSameSide(const Vector2& edgePoint1,
                                const Vector2& edgePoint2,
                                const Vector2& refPoint,
                                const Vector2 targetPoint);

  static bool isCollinear(const Vector2& a, const Vector2& b,
                          const Vector2& c);

  static bool fuzzyIsCollinear(const Vector2& a, const Vector2& b,
                               const Vector2& c);

  static Vector2 PointToLineSegment(const Vector2& a, const Vector2& b,
                                    const Vector2& p);

  static Vector2 lineIntersection(const Vector2& p1, const Vector2& p2,
                                  const Vector2& q1, const Vector2& q2);

  static std::vector<Vector2> GetVertices(
      const CollisionComponent* const body_a);

  ClipEdge static FindClipEdge(const std::span<Vector2>& vertices, size_t index,
                               const Vector2& normal);

  ClipEdge static GetClipEdge(const CollisionComponent& shape,
                              const std::span<Vector2>& vertices,
                              const Vector2& normal);
  static std::vector<PointPair> Clip(const ClipEdge& clipEdgeA,
                                     const ClipEdge& clipEdgeB,
                                     const Vector2& normal);
};
}  // namespace base_engine
