#pragma once

#include <Eigen/Core>

#include <limits>
#include <vector>

namespace collision2d {
template <typename N> using Point = Eigen::Matrix<N, 2, 1>;

template <typename N> using Polygon = std::vector<Point<N>>;

/**
 * Computes potential separating axes for a convex polygon.
 */
template <typename N>
void separatingAxes(const Polygon<N> &a, std::vector<Point<N>> &axes) {
  for (auto i = 0u; i < a.size(); ++i) {
    const auto current = a[i];
    const auto next = a[(i + 1) % a.size()];
    const auto edge = (next - current).normalized();
    // axis is the normal of an edge
    axes.emplace_back(Point<N>{-edge[1], edge[0]});
  }
}

/**
 * Projects the polygon onto the given axis and returns the maximum and minimum
 * coordinate on the axis. Note that the axis has to be normalized.
 */
template <typename N>
void project(const Polygon<N> &a, const Point<N> &axis, N &minProj,
             N &maxProj) {
  maxProj = -std::numeric_limits<N>::infinity();
  minProj = std::numeric_limits<N>::infinity();
  for (const Point<N> &v : a) {
    const N proj = axis.dot(v);
    if (proj < minProj)
      minProj = proj;
    if (proj > maxProj)
      maxProj = proj;
  }
}

/**
 * Check for collision between polygons a and b via the Separating Axis Theorem.
 */
template <typename N> bool intersect(const Polygon<N> &a, const Polygon<N> &b) {
  // compute separating axes
  std::vector<Point<N>> axes;
  separatingAxes(a, axes);
  separatingAxes(b, axes);
  for (const auto &axis : axes) {
    N aMaxProj, aMinProj, bMaxProj, bMinProj;
    project(a, axis, aMinProj, aMaxProj);
    project(b, axis, bMinProj, bMaxProj);
    // check if projections overlap
    if (aMinProj > bMaxProj || bMinProj > aMaxProj)
      return false;
  }

  return true;
}
} // namespace collision2d
