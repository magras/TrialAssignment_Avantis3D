#ifndef TYPES_H
#define TYPES_H

#include <array>
#include <blaze/math/StaticMatrix.h>
#include <blaze/math/StaticVector.h>

using Mat4d = blaze::StaticMatrix<double, 4, 4>;
using Point2d = blaze::StaticVector<double, 2>;
using Point3d = blaze::StaticVector<double, 3>;
using Point4d = blaze::StaticVector<double, 4>;
using Triangle = std::array<Point3d, 3>;
using LineSegment = std::array<Point3d, 2>;

using Triangle2d = std::array<Point2d, 3>;
using LineSegment2d = std::array<Point2d, 2>;

#endif // TYPES_H
