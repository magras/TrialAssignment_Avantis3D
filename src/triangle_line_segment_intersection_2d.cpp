#include <algorithm>
#include <optional>
#include <vector>
#include "geom_types.h"

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

    // If any end of line segment is inside of the triangle return it.
    for (Point2d const& p : ls) {
        if (is_point_inside_triangle(tri, p))
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

// CRAP: This tests are fragile. Probably should check that
// returned point lies inside triangle and line segment.

TEST(TriangleLineSegmentIntersection2d, TouchingVetex) {
    auto x = triangle_line_segment_intersection(
        Triangle2d{
            Point2d{0,0},
            Point2d{0,1},
            Point2d{1,0}},
        LineSegment2d{
            Point2d{-1,1},
            Point2d{1,-1}}
    );
    ASSERT_TRUE(x);
    EXPECT_DOUBLE_EQ(0, x.value()[0]);
    EXPECT_DOUBLE_EQ(0, x.value()[1]);
}

TEST(TriangleLineSegmentIntersection2d, TouchingEdge) {
    auto x = triangle_line_segment_intersection(
        Triangle2d{
            Point2d{0,0},
            Point2d{0,1},
            Point2d{1,0}},
        LineSegment2d{
            Point2d{0.5,0.5},
            Point2d{1,2}}
    );
    ASSERT_TRUE(x);
    EXPECT_DOUBLE_EQ(0.5, x.value()[0]);
    EXPECT_DOUBLE_EQ(0.5, x.value()[1]);
}

TEST(TriangleLineSegmentIntersection2d, WholeLineSegmentInsideTriangle) {
    auto x = triangle_line_segment_intersection(
        Triangle2d{
            Point2d{0,0},
            Point2d{0,4},
            Point2d{4,0}},
        LineSegment2d{
            Point2d{1,2},
            Point2d{2,1}}
    );
    ASSERT_TRUE(x);
    EXPECT_DOUBLE_EQ(1, x.value()[0]);
    EXPECT_DOUBLE_EQ(2, x.value()[1]);
}
