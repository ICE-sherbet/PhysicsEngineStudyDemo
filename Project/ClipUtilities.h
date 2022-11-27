// @ClipUtilities.h
// @brief
// @author ICE
// @date 2022/11/27
//
// @details

#pragma once
#include <vector>

#include "CollisionComponent.h"
#include "IShape.h"
#include "Vector.h"

namespace base_engine {
struct ClipEdge {
  Vector2 p1;
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
  static inline bool SameSign(const float a, const float b) {
    return a >= 0 && b >= 0 || a <= 0 && b <= 0;
  }
  static bool IsPointOnSameSide(const Vector2& edgePoint1,
                                const Vector2& edgePoint2,
                                const Vector2& refPoint,
                                const Vector2 targetPoint) {
    Vector2 u = edgePoint2 - edgePoint1;
    Vector2 v = refPoint - edgePoint1;
    Vector2 w = targetPoint - edgePoint1;
    // same side or on the edge
    return SameSign(VectorUtilities::Cross(u, v), VectorUtilities::Cross(u, w));
  }

  static bool isCollinear(const Vector2& a, const Vector2& b,
                          const Vector2& c) {
    return std::fabs(VectorUtilities::Cross(a - b, a - c) == 0);
  }
  static bool fuzzyIsCollinear(const Vector2& a, const Vector2& b,
                               const Vector2& c) {
    return (c.x <= std::max(a.x, b.x) && c.x >= (std::min)(a.x, b.x) &&
            c.y <= std::max(a.y, b.y) && c.y >= (std::min)(a.y, b.y));
  }
  static Vector2 PointToLineSegment(const Vector2& a, const Vector2& b,
                                    const Vector2& p) {
    // special cases
    if (a == b) return {};

    if (isCollinear(a, b, p)) return p;

    const Vector2 ap = p - a;
    const Vector2 ab_normal = VectorUtilities::Normalize(b - a);
    const Vector2 ap_proj = ab_normal * VectorUtilities::Dot(ab_normal, ap);
    Vector2 op_proj = a + ap_proj;

    if (fuzzyIsCollinear(a, b, op_proj)) return op_proj;
    return VectorUtilities::LengthSquare(p - a) >
                   VectorUtilities::LengthSquare(p - b)
               ? b
               : a;
  }
  static Vector2 lineIntersection(const Vector2& p1, const Vector2& p2,
                                  const Vector2& q1, const Vector2& q2) {
    const float d =
        (p1.x - p2.x) * (q1.y - q2.y) - (p1.y - p2.y) * (q1.x - q2.x);
    if (d == 0) return Vector2();
    const float x = ((p1.x * p2.y - p1.y * p2.x) * (q1.x - q2.x) -
                     (q1.x * q2.y - q1.y * q2.x) * (p1.x - p2.x)) /
                    d;
    const float y = ((p1.x * p2.y - p1.y * p2.x) * (q1.y - q2.y) -
                     (q1.x * q2.y - q1.y * q2.x) * (p1.y - p2.y)) /
                    d;
    return Vector2(x, y);
  }
  static std::vector<Vector2> GetVertices(
      const CollisionComponent* const body_a) {
    std::vector<Vector2> result;
    std::span<Vector2> vertices;
    switch (body_a->GetShape()->GetType()) {
      case ShapeType::kRect: {
        const auto polygon = static_cast<Rect*>(body_a->GetShape());
        vertices = polygon->GetVertices();
        break;
      } 
      case ShapeType::kCircle:
        break;
      case ShapeType::kPoint:
        break;
      default:;
    }
    for (const auto& vertex : vertices) {
      result.emplace_back(vertex + body_a->GetActor()->GetPosition());
    }
    return result;
  }

  ClipEdge static FindClipEdge(const std::span<Vector2>& vertices, size_t index,
                               const Vector2& normal) {
    ClipEdge edge1, edge2;
    edge1.p2 = vertices[index];
    edge2.p1 = vertices[index];
    if (index == 0) {
      edge1.p1 = vertices[vertices.size() - 2];
      edge2.p2 = vertices[index + 1];
    } else if (index == vertices.size() - 1) {
      edge1.p1 = vertices[index - 1];
      edge2.p2 = vertices[1];
    } else {
      edge1.p1 = vertices[index - 1];
      edge2.p2 = vertices[index + 1];
    }
    // compare which is closest to normal
    ClipEdge finalEdge;
    if (std::fabs(VectorUtilities::Dot(edge1.p2 - edge1.p1, normal)) >=
        std::fabs(VectorUtilities::Dot(edge2.p2 - edge2.p1, normal))) {
      finalEdge = edge2;
      Vector2 p = VectorUtilities::Right(
          VectorUtilities::Normalize(edge2.p2 - edge2.p1));
      finalEdge.normal =
          IsPointOnSameSide(edge2.p1, edge2.p2, edge1.p1, edge2.p1 + p) ? p
                                                                        : -p;
    } else {
      finalEdge = edge1;
      Vector2 p = VectorUtilities::Right(
          VectorUtilities::Normalize(edge1.p2 - edge1.p1));
      finalEdge.normal =
          IsPointOnSameSide(edge1.p1, edge1.p2, edge2.p2, edge1.p1 + p) ? p
                                                                        : -p;
    }
    return finalEdge;
  }
  ClipEdge static GetClipEdge(const CollisionComponent& shape,
                              const std::span<Vector2>& vertices,
                              const Vector2& normal) {
    ClipEdge edge;
    if (vertices.size() == 2) {
      edge.p1 = vertices[0];
      edge.p2 = vertices[1];
    } else {
      const auto index = IShape::FindFurthestIndex(
          shape.GetActor()->GetPosition(), vertices, normal);
      edge = FindClipEdge(vertices, index, normal);
    }
    return edge;
  }
  static std::vector<PointPair> Clip(const ClipEdge& clipEdgeA,
                                     const ClipEdge& clipEdgeB,
                                     const Vector2& normal) {
    std::vector<PointPair> result;
    if (clipEdgeA.IsEmpty() || clipEdgeB.IsEmpty()) return result;
    // find reference edge
    float d1 = VectorUtilities::Dot(clipEdgeA.p1 - clipEdgeA.p2, normal);
    float d2 = VectorUtilities::Dot(clipEdgeB.p1 - clipEdgeB.p2, normal);

    ClipEdge referenceEdge = clipEdgeA;
    ClipEdge incidentEdge = clipEdgeB;
    bool swap = false;
    if (std::fabs(d1) > std::fabs(d2)) {
      // edge B is reference edge
      referenceEdge = clipEdgeB;
      incidentEdge = clipEdgeA;
      swap = true;
    }

    // 1. clip left region
    Vector2 u = VectorUtilities::Normalize(referenceEdge.p2 - referenceEdge.p1);
    Vector2 refAnchor1 = VectorUtilities::Right(u) + referenceEdge.p1;
    if (!IsPointOnSameSide(referenceEdge.p1, refAnchor1, referenceEdge.p2,
                           incidentEdge.p1))
      incidentEdge.p1 = lineIntersection(referenceEdge.p1, refAnchor1,
                                         incidentEdge.p1, incidentEdge.p2);
    if (!IsPointOnSameSide(referenceEdge.p1, refAnchor1, referenceEdge.p2,
                           incidentEdge.p2))
      incidentEdge.p2 = lineIntersection(referenceEdge.p1, refAnchor1,
                                         incidentEdge.p1, incidentEdge.p2);

    // 2. clip right region
    u = -u;
    Vector2 refAnchor2 = VectorUtilities::Right(u) + referenceEdge.p2;
    if (!IsPointOnSameSide(referenceEdge.p2, refAnchor2, referenceEdge.p1,
                           incidentEdge.p1))
      incidentEdge.p1 = lineIntersection(referenceEdge.p2, refAnchor2,
                                         incidentEdge.p1, incidentEdge.p2);
    if (!IsPointOnSameSide(referenceEdge.p2, refAnchor2, referenceEdge.p1,
                           incidentEdge.p2))
      incidentEdge.p2 = lineIntersection(referenceEdge.p2, refAnchor2,
                                         incidentEdge.p1, incidentEdge.p2);

    // 3. clip normal region
    Vector2 refAnchor3 =
        (referenceEdge.p2 + referenceEdge.p1) / 2.0f + referenceEdge.normal;

    bool p1OnClipArea = IsPointOnSameSide(referenceEdge.p1, referenceEdge.p2,
                                          refAnchor3, incidentEdge.p1);
    bool p2OnClipArea = IsPointOnSameSide(referenceEdge.p1, referenceEdge.p2,
                                          refAnchor3, incidentEdge.p2);

    if (!(p1OnClipArea && p2OnClipArea)) return result;

    if (p1OnClipArea && !p2OnClipArea)  // p1 inside, p2 outside
      incidentEdge.p2 = lineIntersection(referenceEdge.p1, referenceEdge.p2,
                                         incidentEdge.p1, incidentEdge.p2);

    else if (!p1OnClipArea && p2OnClipArea)  // p1 outside, p2 inside
      incidentEdge.p1 = lineIntersection(referenceEdge.p1, referenceEdge.p2,
                                         incidentEdge.p1, incidentEdge.p2);

    // p1 and p2 are inside, clip nothing, just go to project
    // 4. project to reference edge
    Vector2 pp1 =
        PointToLineSegment(referenceEdge.p1, referenceEdge.p2, incidentEdge.p1);
    Vector2 pp2 =
        PointToLineSegment(referenceEdge.p1, referenceEdge.p2, incidentEdge.p2);
    result.reserve(2);
    PointPair pair1, pair2;
    if (!swap) {
      pair1.pointA = pp1;
      pair1.pointB = incidentEdge.p1;
      pair2.pointA = pp2;
      pair2.pointB = incidentEdge.p2;
    } else {
      pair1.pointA = incidentEdge.p1;
      pair1.pointB = pp1;
      pair2.pointA = incidentEdge.p2;
      pair2.pointB = pp2;
    }
    result.emplace_back(pair1);
    result.emplace_back(pair2);
    return result;
  }
};
}  // namespace base_engine
