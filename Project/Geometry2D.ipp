#pragma once
#include <Collision/Circle.h>
#include <Collision/Rectangle.h>
#include <Math/Vector2.h>
namespace base_engine::Geometry2D {
namespace detail {
inline float DistanceSqrf(const float t_x1, const float t_y1, const float t_x2,
                   const float t_y2) {
  float dx = t_x2 - t_x1;
  float dy = t_y2 - t_y1;

  return (dx * dx) + (dy * dy);
}
inline bool IntersectCircleToRect(const Mof::CCircle& a,
                      const Mof::CRectangle& b) noexcept {
  bool nResult = false;

  // 四角形の四辺に対して円の半径分だけ足したとき円が重なっていたら
  if ((a.x > b.Left - a.r) && (a.x < b.Right + a.r) && (a.y > b.Top - a.r) &&
      (a.y < b.Bottom + a.r)) {
    nResult = true;
    float fl = a.r * a.r;

    // 左
    if (a.x < b.Left) {
      // 左上
      if ((a.y < b.Top)) {
        if ((detail::DistanceSqrf(b.Left, b.Top, a.x, a.y) >= fl)) {
          nResult = false;
        }
      } else {
        // 左下
        if ((a.y > b.Bottom)) {
          if ((detail::DistanceSqrf(b.Left, b.Bottom, a.x, a.y) >= fl)) {
            nResult = false;
          }
        }
      }
    } else {
      // 右
      if (a.x > b.Right) {
        // 右上
        if ((a.y < b.Top)) {
          if ((detail::DistanceSqrf(b.Right, b.Top, a.x, a.y) >= fl)) {
            nResult = false;
          }
        } else {
          // 右下
          if ((a.y > b.Bottom)) {
            if ((detail::DistanceSqrf(b.Right, b.Bottom, a.x, a.y) >= fl)) {
              nResult = false;
            }
          }
        }
      }
    }
  }

  return nResult;
};
}  // namespace detail
//////////////////////////////////////////////////
///
//          Mof::CVector2
///
//////////////////////////////////////////////////
inline bool Intersect(const Mof::CVector2& a, const Mof::CVector2& b) noexcept {
  return (a == b);
}
inline bool Intersect(const Mof::CVector2& a,
                      const Mof::CRectangle& b) noexcept {
  return b.CollisionPoint(a);
};
inline bool Intersect(const Mof::CVector2& a, const Mof::CCircle& b) noexcept {
  return b.CollisionPoint(a);
}
//////////////////////////////////////////////////
///
//          Mof::CRectangle
///
//////////////////////////////////////////////////
inline bool Intersect(const Mof::CRectangle& a,
                      const Mof::CVector2& b) noexcept {
  return a.CollisionPoint(b);
}
inline bool Intersect(const Mof::CRectangle& a,
                      const Mof::CRectangle& b) noexcept {
  return a.CollisionRect(b);
};
inline bool Intersect(const Mof::CRectangle& a,
                      const Mof::CCircle& b) noexcept {
  return detail::IntersectCircleToRect(b, a);
}
//////////////////////////////////////////////////
///
//          Mof::CCircle
///
//////////////////////////////////////////////////
inline bool Intersect(const Mof::CCircle& a, const Mof::CVector2& b) noexcept {
  return a.CollisionPoint(b);
}

inline bool Intersect(const Mof::CCircle& a,
                      const Mof::CRectangle& b) noexcept {
  return detail::IntersectCircleToRect(a,b);
};
inline bool Intersect(const Mof::CCircle& a, const Mof::CCircle& b) noexcept {
  return a.CollisionCircle(b);
}

}  // namespace base_engine::Geometry2D
