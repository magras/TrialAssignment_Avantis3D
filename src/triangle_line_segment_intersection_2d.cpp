#include <optional>
#include <range/v3/algorithm/all_of.hpp>
#include "equal_to_zero.h"
#include "geom_types.h"

namespace {

    inline double perp_dot(Point2d const& a, Point2d const& b) {
        return a[0] * b[1] - a[1] * b[0];
    }

    bool is_point_on_line_segment(Point2d const& p, LineSegment2d const& ls) {
        
        Point2d a = ls[1] - ls[0];
        Point2d b = p - ls[0];
        double n = sqrNorm(a);

        // Check that point is on a line.
        if (not_equal_to_zero(perp_dot(a, b), n))
            return false;

        // Check that point is inside line segment.
        double x = dot(a, b);
        return 0 <= x && x <= n;
    }

    bool is_point_inside_triangle(Point2d const& p, Triangle2d const& t) {
        std::array<double, 3> s {
            perp_dot(t[1] - t[0], p - t[0]),
            perp_dot(t[2] - t[1], p - t[1]),
            perp_dot(t[0] - t[2], p - t[2])};

        return ranges::all_of(s, [](double x){ return x >= 0; })
            || ranges::all_of(s, [](double x){ return x <= 0; });
    }

} // anonymous namespace

std::optional<Point2d> line_segment_line_segment_intersection(LineSegment2d const& ls0, LineSegment2d const& ls1);

std::optional<Point2d> triangle_line_segment_intersection(Triangle2d const& tri, LineSegment2d const& ls) {

    // If any end of line segment is inside of the triangle return it.
    for (Point2d const& p : ls) {
        if (is_point_inside_triangle(p, tri))
            return p;
    }

    // If there is an intersection between line segment and any of the
    // triangle sides return intersection point.
    for (size_t i=0; i<tri.size(); ++i) {
        LineSegment2d edge{ tri[i], tri[i % tri.size()] };
        auto x = line_segment_line_segment_intersection(edge, ls);
        if (x.has_value())
            return x.value();
    }

    return std::nullopt;
}

#include <gtest/gtest.h>

TEST(TriangleLineSegmentIntersection2d, NoIntersection) {
    auto t = Triangle2d{
        Point2d{0,0},
        Point2d{0,1},
        Point2d{1,0}};
    auto ls = LineSegment2d{
        Point2d{0,2},
        Point2d{2,0}};
    auto x = triangle_line_segment_intersection(t, ls);
    ASSERT_FALSE(x);
}

TEST(TriangleLineSegmentIntersection2d, TouchingVetex) {
    auto t = Triangle2d{
        Point2d{0,0},
        Point2d{0,1},
        Point2d{1,0}};
    auto ls = LineSegment2d{
        Point2d{-1,1},
        Point2d{1,-1}};
    auto x = triangle_line_segment_intersection(t, ls);
    ASSERT_TRUE(x);
    ASSERT_TRUE(is_point_inside_triangle(x.value(), t));
    ASSERT_TRUE(is_point_on_line_segment(x.value(), ls));
}

TEST(TriangleLineSegmentIntersection2d, TouchingEdge) {
    auto t = Triangle2d{
        Point2d{0,0},
        Point2d{0,1},
        Point2d{1,0}};
    auto ls = LineSegment2d{
        Point2d{0.5,0.5},
        Point2d{1,2}};
    auto x = triangle_line_segment_intersection(t, ls);
    ASSERT_TRUE(x);
    ASSERT_TRUE(is_point_inside_triangle(x.value(), t));
    ASSERT_TRUE(is_point_on_line_segment(x.value(), ls));
}

TEST(TriangleLineSegmentIntersection2d, WholeLineSegmentInsideTriangle) {
    auto t = Triangle2d{
        Point2d{0,0},
        Point2d{0,4},
        Point2d{4,0}};
    auto ls = LineSegment2d{
        Point2d{1,2},
        Point2d{2,1}};
    auto x = triangle_line_segment_intersection(t, ls);
    ASSERT_TRUE(x);
    ASSERT_TRUE(is_point_inside_triangle(x.value(), t));
    ASSERT_TRUE(is_point_on_line_segment(x.value(), ls));
}
