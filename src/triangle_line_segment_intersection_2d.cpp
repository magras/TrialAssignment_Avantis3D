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

    // Basic idea of this algorithm is to find all points that intersect triangle
    // and build "convex hull" for this set. It would allow to build line segment
    // that represent all points of intersection.
    
    // If it's enough to return just any point, replace all
    // `intersection.push_back()` calls with early returns.

    // I like my algorithm, because it has no special cases, hence it's easy to
    // implement.

    std::vector<Point2d> intersection;
    intersection.reserve(5);

    // Add line segment ends to set, if they are inside of a triangle.
    for (Point2d const& p : ls) {
        if (is_point_inside_triangle(tri, p))
            intersection.push_back(p);
    }

    // Find and add to set intersections between all triangle's edges and line segment.
    for (size_t i=0; i<tri.size(); ++i) {
        LineSegment2d edge{ tri[i], tri[i % tri.size()] };
        auto x = line_segment_line_segment_intersection(edge, ls);
        if (x)
            intersection.push_back(*x);
    }

    // Remove duplicates from the set.
    // I'm assuming here, that duplicates would be exact match, because duplicates should
    // appear in a set only if some of input data got duplicates in it. Therefore
    // line segment - line segment intersection algorithm will return them without any
    // modifications.
    std::sort(begin(intersection), end(intersection), [](Point2d const& lhs, Point2d const& rhs){
        return std::lexicographical_compare(begin(lhs), end(lhs), begin(rhs), end(rhs));
    });
    intersection.erase(
        std::unique(begin(intersection), end(intersection)),
        end(intersection));

    // After removing duplicates there should be 0, 1, or 2 points in a set.
    assert(intersection.size() >= 0);
    assert(intersection.size() <= 2);

    if (intersection.empty())
        return std::nullopt;

    // Instead of possibly building a line segment just return any point from the set.
    return intersection[0];
}

#include <gtest/gtest.h>

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

// DEBUG: It's hard to test this function properly. I think I'll change
//        return type to allow line segment results.
