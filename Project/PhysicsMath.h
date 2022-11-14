// @PhysicsMath.h
// @brief
// @author ICE
// @date 2022/11/14
//
// @details

#pragma once
#include <corecrt_math.h>

#include <numbers>

#include "PhysicsTypes.h"
#include "Vector.h"
namespace base_engine::physics {

/// A 2D column vector.
struct PhysicsVec2 {
  /// Default constructor does nothing (for performance).
  PhysicsVec2() = default;

  /// Construct using coordinates.
  PhysicsVec2(float xIn, float yIn) : x(xIn), y(yIn) {}

  /// Set this vector to all zeros.
  void SetZero() {
    x = 0.0f;
    y = 0.0f;
  }

  /// Set this vector to some specified coordinates.
  void Set(float x_, float y_) {
    x = x_;
    y = y_;
  }

  /// Negate this vector.
  PhysicsVec2 operator-() const {
    PhysicsVec2 v;
    v.Set(-x, -y);
    return v;
  }

  /// Read from and indexed element.
  float operator()(int32 i) const { return (&x)[i]; }

  /// Write to an indexed element.
  float& operator()(int32 i) { return (&x)[i]; }

  /// Add a vector to this vector.
  void operator+=(const PhysicsVec2& v) noexcept {
    x += v.x;
    y += v.y;
  }

  /// Subtract a vector from this vector.
  void operator-=(const PhysicsVec2& v) noexcept {
    x -= v.x;
    y -= v.y;
  }

  /// Multiply this vector by a scalar.
  void operator*=(float a) noexcept {
    x *= a;
    y *= a;
  }
  PhysicsVec2 operator=(InVector2 v) {
    this->x = v.x;
    this->y = v.y;
    return *this;
  }
  /// Get the length of this vector (the norm).
  [[nodiscard]] float Length() const noexcept { return sqrtf(x * x + y * y); }

  /// Get the length squared. For performance, use this instead of
  /// PhysicsVec2::Length (if possible).
  [[nodiscard]] float LengthSquared() const noexcept { return x * x + y * y; }

  /// Convert this vector into a unit vector. Returns the length.
  float Normalize() {
    const float length = Length();
    if (length < std::numeric_limits<float>::epsilon()) {
      return 0.0f;
    }
    const float inv_length = 1.0f / length;
    x *= inv_length;
    y *= inv_length;

    return length;
  }

  /// Does this vector contain finite coordinates?
  [[nodiscard]] bool IsValid() const noexcept {
    return isfinite(x) && isfinite(y);
  }

  /// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
  [[nodiscard]] PhysicsVec2 Skew() const noexcept { return {-y, x}; }

  float x, y;
};

/// A 2D column vector with 3 elements.
struct PhysicsVec3 {
  /// Default constructor does nothing (for performance).
  PhysicsVec3() = default;

  /// Construct using coordinates.
  PhysicsVec3(float xIn, float yIn, float zIn) : x(xIn), y(yIn), z(zIn) {}

  /// Set this vector to all zeros.
  void SetZero() {
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
  }

  /// Set this vector to some specified coordinates.
  void Set(float x_, float y_, float z_) {
    x = x_;
    y = y_;
    z = z_;
  }

  /// Negate this vector.
  PhysicsVec3 operator-() const {
    PhysicsVec3 v;
    v.Set(-x, -y, -z);
    return v;
  }

  /// Add a vector to this vector.
  void operator+=(const PhysicsVec3& v) {
    x += v.x;
    y += v.y;
    z += v.z;
  }

  /// Subtract a vector from this vector.
  void operator-=(const PhysicsVec3& v) {
    x -= v.x;
    y -= v.y;
    z -= v.z;
  }

  /// Multiply this vector by a scalar.
  void operator*=(float s) {
    x *= s;
    y *= s;
    z *= s;
  }

  float x, y, z;
};

/// A 2-by-2 matrix. Stored in column-major order.
struct PhysicsMat22 {
  /// The default constructor does nothing (for performance).
  PhysicsMat22() = default;

  /// Construct this matrix using columns.
  PhysicsMat22(const PhysicsVec2& c1, const PhysicsVec2& c2) {
    ex = c1;
    ey = c2;
  }

  /// Construct this matrix using scalars.
  PhysicsMat22(float a11, float a12, float a21, float a22) {
    ex.x = a11;
    ex.y = a21;
    ey.x = a12;
    ey.y = a22;
  }

  /// Initialize this matrix using columns.
  void Set(const PhysicsVec2& c1, const PhysicsVec2& c2) {
    ex = c1;
    ey = c2;
  }

  /// Set this to the identity matrix.
  void SetIdentity() {
    ex.x = 1.0f;
    ey.x = 0.0f;
    ex.y = 0.0f;
    ey.y = 1.0f;
  }

  /// Set this matrix to all zeros.
  void SetZero() {
    ex.x = 0.0f;
    ey.x = 0.0f;
    ex.y = 0.0f;
    ey.y = 0.0f;
  }

  PhysicsMat22 GetInverse() const {
    float a = ex.x, b = ey.x, c = ex.y, d = ey.y;
    PhysicsMat22 B;
    float det = a * d - b * c;
    if (det != 0.0f) {
      det = 1.0f / det;
    }
    B.ex.x = det * d;
    B.ey.x = -det * b;
    B.ex.y = -det * c;
    B.ey.y = det * a;
    return B;
  }

  /// Solve A * x = b, where b is a column vector. This is more efficient
  /// than computing the inverse in one-shot cases.
  [[nodiscard]] PhysicsVec2 Solve(const PhysicsVec2& b) const {
    float a11 = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
    float det = a11 * a22 - a12 * a21;
    if (det != 0.0f) {
      det = 1.0f / det;
    }
    PhysicsVec2 x;
    x.x = det * (a22 * b.x - a12 * b.y);
    x.y = det * (a11 * b.y - a21 * b.x);
    return x;
  }

  PhysicsVec2 ex, ey;
};

/// A 3-by-3 matrix. Stored in column-major order.
struct PhysicsMat33 {
  /// The default constructor does nothing (for performance).
  PhysicsMat33() = default;

  /// Construct this matrix using columns.
  PhysicsMat33(const PhysicsVec3& c1, const PhysicsVec3& c2,
               const PhysicsVec3& c3) {
    ex = c1;
    ey = c2;
    ez = c3;
  }

  /// Set this matrix to all zeros.
  void SetZero() {
    ex.SetZero();
    ey.SetZero();
    ez.SetZero();
  }

  /// Solve A * x = b, where b is a column vector. This is more efficient
  /// than computing the inverse in one-shot cases.
  PhysicsVec3 Solve33(const PhysicsVec3& b) const;

  /// Solve A * x = b, where b is a column vector. This is more efficient
  /// than computing the inverse in one-shot cases. Solve only the upper
  /// 2-by-2 matrix equation.
  PhysicsVec2 Solve22(const PhysicsVec2& b) const;

  /// Get the inverse of this matrix as a 2-by-2.
  /// Returns the zero matrix if singular.
  void GetInverse22(PhysicsMat33* M) const;

  /// Get the symmetric inverse of this matrix as a 3-by-3.
  /// Returns the zero matrix if singular.
  void GetSymInverse33(PhysicsMat33* M) const;

  PhysicsVec3 ex, ey, ez;
};

struct PhysicsRot {
  PhysicsRot() = default;

  /// Initialize from an angle in radians
  explicit PhysicsRot(float angle) {
    /// TODO_ERIN optimize
    s = sinf(angle);
    c = cosf(angle);
  }

  /// Set using an angle in radians.
  void Set(float angle) {
    /// TODO_ERIN optimize
    s = sinf(angle);
    c = cosf(angle);
  }

  /// Set to the identity rotation
  void SetIdentity() {
    s = 0.0f;
    c = 1.0f;
  }

  /// Get the angle in radians
  [[nodiscard]] float GetAngle() const { return atan2(s, c); }

  /// Get the x-axis
  PhysicsVec2 GetXAxis() const { return PhysicsVec2(c, s); }

  /// Get the u-axis
  PhysicsVec2 GetYAxis() const { return PhysicsVec2(-s, c); }

  /// Sine and cosine
  float s, c;
};

/// A transform contains translation and rotation. It is used to represent
/// the position and orientation of rigid frames.
struct PhysicsTransform {
  /// The default constructor does nothing.
  PhysicsTransform() = default;

  /// Initialize using a position vector and a rotation.
  PhysicsTransform(const PhysicsVec2& position, const PhysicsRot& rotation)
      : p(position), q(rotation) {}

  /// Set this to the identity transform.
  void SetIdentity() {
    p;
    q.SetIdentity();
  }

  /// Set this based on the position and angle.
  void Set(const PhysicsVec2& position, float angle) {
    p = position;
    q.Set(angle);
  }

  PhysicsVec2 p;
  PhysicsRot q;
};
struct PhysicsSweep {
  PhysicsSweep();

  /// Get the interpolated transform at a specific time.
  /// @param transform the output transform
  /// @param beta is a factor in [0,1], where 0 indicates alpha0.
  void GetTransform(PhysicsTransform* xf, float beta) const;

  /// Advance the sweep forward, yielding a new initial state.
  /// @param alpha the new initial time.
  void Advance(float alpha);

  void Normalize();

  PhysicsVec2 localCenter;  ///< local center of mass position
  PhysicsVec2 c0, c;        ///< center world positions
  float a0, a;              ///< world angles

  /// Fraction of the current time step in the range [0,1]
  /// c0 and a0 are the positions at alpha0.
  float alpha0;
};

/// Useful constant
extern const PhysicsVec2 PhysicsVec2_zero;

/// Perform the dot product on two vectors.
inline float PhysicsDot(const PhysicsVec2& a, const PhysicsVec2& b) {
  return a.x * b.x + a.y * b.y;
}

/// Perform the cross product on two vectors. In 2D this produces a scalar.
inline float PhysicsCross(const PhysicsVec2& a, const PhysicsVec2& b) {
  return a.x * b.y - a.y * b.x;
}

/// Perform the cross product on a vector and a scalar. In 2D this produces
/// a vector.
inline PhysicsVec2 PhysicsCross(const PhysicsVec2& a, float s) {
  return PhysicsVec2(s * a.y, -s * a.x);
}

/// Perform the cross product on a scalar and a vector. In 2D this produces
/// a vector.
inline PhysicsVec2 PhysicsCross(float s, const PhysicsVec2& a) {
  return PhysicsVec2(-s * a.y, s * a.x);
}

/// Multiply a matrix times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another.
inline PhysicsVec2 PhysicsMul(const PhysicsMat22& A, const PhysicsVec2& v) {
  return PhysicsVec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
}

/// Multiply a matrix transpose times a vector. If a rotation matrix is
/// provided, then this transforms the vector from one frame to another (inverse
/// transform).
inline PhysicsVec2 PhysicsMulT(const PhysicsMat22& A, const PhysicsVec2& v) {
  return PhysicsVec2(PhysicsDot(v, A.ex), PhysicsDot(v, A.ey));
}

/// Add two vectors component-wise.
inline PhysicsVec2 operator+(const PhysicsVec2& a, const PhysicsVec2& b) {
  return PhysicsVec2(a.x + b.x, a.y + b.y);
}

/// Subtract two vectors component-wise.
inline PhysicsVec2 operator-(const PhysicsVec2& a, const PhysicsVec2& b) {
  return PhysicsVec2(a.x - b.x, a.y - b.y);
}

inline PhysicsVec2 operator*(float s, const PhysicsVec2& a) {
  return PhysicsVec2(s * a.x, s * a.y);
}

inline bool operator==(const PhysicsVec2& a, const PhysicsVec2& b) {
  return a.x == b.x && a.y == b.y;
}



inline bool operator!=(const PhysicsVec2& a, const PhysicsVec2& b) {
  return a.x != b.x || a.y != b.y;
}

inline float PhysicsDistance(const PhysicsVec2& a, const PhysicsVec2& b) {
  PhysicsVec2 c = a - b;
  return c.Length();
}

inline float PhysicsDistanceSquared(const PhysicsVec2& a,
                                    const PhysicsVec2& b) {
  PhysicsVec2 c = a - b;
  return PhysicsDot(c, c);
}

inline PhysicsVec3 operator*(float s, const PhysicsVec3& a) {
  return {s * a.x, s * a.y, s * a.z};
}

/// Add two vectors component-wise.
inline PhysicsVec3 operator+(const PhysicsVec3& a, const PhysicsVec3& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

/// Subtract two vectors component-wise.
inline PhysicsVec3 operator-(const PhysicsVec3& a, const PhysicsVec3& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

/// Perform the dot product on two vectors.
inline float PhysicsDot(const PhysicsVec3& a, const PhysicsVec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// Perform the cross product on two vectors.
inline PhysicsVec3 PhysicsCross(const PhysicsVec3& a, const PhysicsVec3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline PhysicsMat22 operator+(const PhysicsMat22& A, const PhysicsMat22& B) {
  return PhysicsMat22(A.ex + B.ex, A.ey + B.ey);
}

// A * B
inline PhysicsMat22 PhysicsMul(const PhysicsMat22& A, const PhysicsMat22& B) {
  return PhysicsMat22(PhysicsMul(A, B.ex), PhysicsMul(A, B.ey));
}

// A^T * B
inline PhysicsMat22 PhysicsMulT(const PhysicsMat22& A, const PhysicsMat22& B) {
  PhysicsVec2 c1(PhysicsDot(A.ex, B.ex), PhysicsDot(A.ey, B.ex));
  PhysicsVec2 c2(PhysicsDot(A.ex, B.ey), PhysicsDot(A.ey, B.ey));
  return {c1, c2};
}

inline PhysicsVec3 PhysicsMul(const PhysicsMat33& A, const PhysicsVec3& v) {
  return v.x * A.ex + v.y * A.ey + v.z * A.ez;
}

inline PhysicsVec2 PhysicsMul22(const PhysicsMat33& A, const PhysicsVec2& v) {
  return PhysicsVec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
}

/// Multiply two rotations: q * r
inline PhysicsRot PhysicsMul(const PhysicsRot& q, const PhysicsRot& r) {
  // [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
  // [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
  // s = qs * rc + qc * rs
  // c = qc * rc - qs * rs
  PhysicsRot qr{};
  qr.s = q.s * r.c + q.c * r.s;
  qr.c = q.c * r.c - q.s * r.s;
  return qr;
}

/// Transpose multiply two rotations: qT * r
inline PhysicsRot PhysicsMulT(const PhysicsRot& q, const PhysicsRot& r) {
  // [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
  // [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
  // s = qc * rs - qs * rc
  // c = qc * rc + qs * rs
  PhysicsRot qr;
  qr.s = q.c * r.s - q.s * r.c;
  qr.c = q.c * r.c + q.s * r.s;
  return qr;
}

/// Rotate a vector
inline PhysicsVec2 PhysicsMul(const PhysicsRot& q, const PhysicsVec2& v) {
  return {q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y};
}

/// Inverse rotate a vector
inline PhysicsVec2 PhysicsMulT(const PhysicsRot& q, const PhysicsVec2& v) {
  return {q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y};
}

inline PhysicsVec2 PhysicsMul(const PhysicsTransform& T, const PhysicsVec2& v) {
  const float x = (T.q.c * v.x - T.q.s * v.y) + T.p.x;
  const float y = (T.q.s * v.x + T.q.c * v.y) + T.p.y;

  return {x, y};
}

inline PhysicsVec2 PhysicsMulT(const PhysicsTransform& T,
                               const PhysicsVec2& v) {
  const float px = v.x - T.p.x;
  const float py = v.y - T.p.y;
  const float x = (T.q.c * px + T.q.s * py);
  const float y = (-T.q.s * px + T.q.c * py);

  return {x, y};
}

// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
//    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
inline PhysicsTransform PhysicsMul(const PhysicsTransform& A,
                                   const PhysicsTransform& B) {
  PhysicsTransform C;
  C.q = PhysicsMul(A.q, B.q);
  C.p = PhysicsMul(A.q, B.p) + A.p;
  return C;
}

// v2 = A.q' * (B.q * v1 + B.p - A.p)
//    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
inline PhysicsTransform PhysicsMulT(const PhysicsTransform& A,
                                    const PhysicsTransform& B) {
  PhysicsTransform C;
  C.q = PhysicsMulT(A.q, B.q);
  C.p = PhysicsMulT(A.q, B.p - A.p);
  return C;
}

template <typename T>
inline T PhysicsAbs(T a) {
  return a > T(0) ? a : -a;
}

inline PhysicsVec2 PhysicsAbs(const PhysicsVec2& a) {
  return PhysicsVec2(PhysicsAbs(a.x), PhysicsAbs(a.y));
}

inline PhysicsMat22 PhysicsAbs(const PhysicsMat22& A) {
  return PhysicsMat22(PhysicsAbs(A.ex), PhysicsAbs(A.ey));
}

template <typename T>
inline T PhysicsMin(T a, T b) {
  return a < b ? a : b;
}

inline PhysicsVec2 PhysicsMin(const PhysicsVec2& a, const PhysicsVec2& b) {
  return PhysicsVec2(PhysicsMin(a.x, b.x), PhysicsMin(a.y, b.y));
}

template <typename T>
inline T PhysicsMax(T a, T b) {
  return a > b ? a : b;
}

inline PhysicsVec2 PhysicsMax(const PhysicsVec2& a, const PhysicsVec2& b) {
  return PhysicsVec2(PhysicsMax(a.x, b.x), PhysicsMax(a.y, b.y));
}

template <typename T>
inline T PhysicsClamp(T a, T low, T high) {
  return PhysicsMax(low, PhysicsMin(a, high));
}

inline PhysicsVec2 PhysicsClamp(const PhysicsVec2& a, const PhysicsVec2& low,
                                const PhysicsVec2& high) {
  return PhysicsMax(low, PhysicsMin(a, high));
}

template <typename T>
inline void PhysicsSwap(T& a, T& b) {
  T tmp = a;
  a = b;
  b = tmp;
}

inline PhysicsSweep::PhysicsSweep() = default;

inline void PhysicsSweep::GetTransform(PhysicsTransform* xf, float beta) const {
  xf->p = (1.0f - beta) * c0 + beta * c;
  const float angle = (1.0f - beta) * a0 + beta * a;
  xf->q.Set(angle);

  xf->p -= PhysicsMul(xf->q, localCenter);
}

inline void PhysicsSweep::Advance(const float alpha) {
  const float beta = (alpha - alpha0) / (1.0f - alpha0);
  c0 += beta * (c - c0);
  a0 += beta * (a - a0);
  alpha0 = alpha;
}

inline void PhysicsSweep::Normalize() {
  constexpr float two_pi = 2.0f * std::numbers::pi;
  float d = two_pi * floorf(a0 / two_pi);
  a0 -= d;
  a -= d;
}
}  // namespace base_engine::physics