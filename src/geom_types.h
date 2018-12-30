#ifndef GEOM_TYPES_H
#define GEOM_TYPES_H

#include <array>
#include <blaze/math/StaticMatrix.h>
#include <blaze/math/StaticVector.h>

using Mat4d = blaze::StaticMatrix<double, 4, 4>;

template <size_t N>
using Point = blaze::StaticVector<double, N>;
using Point2d = Point<2>;
using Point3d = Point<3>;
using Point4d = Point<4>;

template <size_t N>
using LineSegment = std::array<Point<N>, 2>;
using LineSegment2d = LineSegment<2>;
using LineSegment3d = LineSegment<3>;
using LineSegment4d = LineSegment<4>;

template <size_t N>
using Triangle = std::array<Point<N>, 3>;
using Triangle2d = Triangle<2>;
using Triangle3d = Triangle<3>;

#endif // GEOM_TYPES_H
