#pragma once

#include <cmath>

namespace cmis::orca {

struct Vector2 {
  double x{0.0};
  double y{0.0};

  [[nodiscard]] constexpr Vector2 operator+(const Vector2 &other) const noexcept {
    return {x + other.x, y + other.y};
  }

  [[nodiscard]] constexpr Vector2 operator-(const Vector2 &other) const noexcept {
    return {x - other.x, y - other.y};
  }

  [[nodiscard]] constexpr Vector2 operator*(double scalar) const noexcept {
    return {x * scalar, y * scalar};
  }

  constexpr Vector2 &operator+=(const Vector2 &other) noexcept {
    x += other.x;
    y += other.y;
    return *this;
  }

  [[nodiscard]] double norm() const noexcept {
    return std::sqrt(x * x + y * y);
  }
};

}  // namespace cmis::orca
