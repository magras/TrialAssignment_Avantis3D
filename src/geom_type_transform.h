#ifndef GEOM_TYPE_TRANSFORM_H
#define GEOM_TYPE_TRANSFORM_H

#include "geom_types.h"

template <typename T>
struct number_of_dimentions;

template <size_t N>
struct number_of_dimentions<Point<N>> : std::integral_constant<size_t, N> {};

template <size_t N>
struct number_of_dimentions<LineSegment<N>> : std::integral_constant<size_t, N> {};

template <size_t N>
struct number_of_dimentions<Triangle<N>> : std::integral_constant<size_t, N> {};

template <typename T>
inline constexpr size_t number_of_dimentions_v = number_of_dimentions<T>::value;


inline Point2d point_to_2d(Point3d const& x) {
    assert(x[2] == 0);
    return Point2d{ x[0], x[1] };
}

inline Point3d point_to_3d(Point2d const& x) {
    return Point3d{ x[0], x[1], 0 };
}


template <typename F, size_t V, size_t N, size_t M = number_of_dimentions_v<std::invoke_result_t<F, Point<N> const&>>>
std::array<Point<M>, V> transform(std::array<Point<N>, V> const& ps, F f) {
    std::array<Point<M>, V> res;
    std::transform(begin(ps), end(ps), begin(res), f);
    return res;
}

#endif // GEOM_TYPE_TRANSFORM_H
