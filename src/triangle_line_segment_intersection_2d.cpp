#include <algorithm>
#include <optional>
#include <vector>
#include "types.h"

namespace {

    inline double perp_dot(Point2d const& a, Point2d const& b) {
        return a[0] * b[1] - a[1] * b[0];
    }

    bool is_point_inside_triangle(Triangle2d const& t, Point2d const& p) {
        std::array<double, 3> s {
            perp_dot(t[1] - t[0], p - t[0]),
            perp_dot(t[2] - t[1], p - t[1]),
            perp_dot(t[0] - t[2], p - t[2])};

        return std::all_of(begin(s), end(s), [](double x){ return x >= 0; })
            || std::all_of(begin(s), end(s), [](double x){ return x <= 0; });
    }

} // anonymous namespace

std::optional<Point2d> line_segment_line_segment_intersection(LineSegment2d const& ls0, LineSegment2d const& ls1);

std::optional<Point2d> triangle_line_segment_intersection(Triangle2d const& tri, LineSegment2d const& ls) {

    std::vector<Point2d> intersection;

    for (Point2d const& p : ls) {
        if (is_point_inside_triangle(tri, p))
            intersection.push_back(p);
    }

    for (size_t i=0; i<tri.size(); ++i) {
        LineSegment2d edge{ tri[i], tri[i % tri.size()] };
        auto x = line_segment_line_segment_intersection(edge, ls);
        if (x)
            intersection.push_back(*x);
    }

    std::sort(begin(intersection), end(intersection), [](Point2d const& lhs, Point2d const& rhs){
        return std::lexicographical_compare(begin(lhs), end(lhs), begin(rhs), end(rhs));
    });
    intersection.erase(
        std::unique(begin(intersection), end(intersection)),
        end(intersection));

    assert(intersection.size() >= 0);
    assert(intersection.size() <= 2);

    if (intersection.empty())
        return std::nullopt;

    return intersection[0];
}
