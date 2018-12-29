#include <algorithm>
#include <optional>
#include "equal_to_zero.h"
#include "types.h"

// There is proposal P0627 about [[unreachable]] attribute.
// Actually `assert(false)` has different meaning, but it's
// platform independent and "good enough" for prototype code.
#define UNREACHABLE assert(false)

std::optional<Point3d> line_segment_line_segment_intersection(LineSegment ln0, LineSegment ln1);
std::optional<Point2d> triangle_line_segment_intersection(Triangle2d const& tri, LineSegment2d const& ls);

namespace {

    Point4d to_homogen(Point3d const& p) {
        return Point4d{p[0], p[1], p[2], 1};
    }

    Point3d from_homogen(Point4d const& p) {
        return Point3d{
            p[0] / p[3],
            p[1] / p[3],
            p[2] / p[3]};
    }

    Point3d perpendicular(Triangle const& t) {
        return cross(t[1]-t[0], t[2]-t[0]);
    }

    Mat4d transformation_matrix(Point3d const& i, Point3d const& j, Point3d const& k, Point3d const& t) {
        return Mat4d{
            {i[0], j[0], k[0], t[0]},
            {i[1], j[1], k[1], t[1]},
            {i[2], j[2], k[2], t[2]},
            {0., 0., 0., 1.}};
    }

    Point3d mid_point(Point3d const& a, Point3d const& b, double k) {
        return a + k * (b - a);
    }

    LineSegment degenerate_triangle_to_line_segment(Triangle const& t) {

        auto a = norm(t[0] - t[1]);
        auto b = norm(t[1] - t[2]);
        auto c = norm(t[2] - t[0]);

        if (a >= b && a >= c)
            return {t[0], t[1]};
        if (b >= a && b >= c)
            return {t[1], t[2]};
        if (c >= a && c >= b)
            return {t[2], t[0]};

        UNREACHABLE;
    }

} // anonymous namespace

std::optional<Point3d> triangle_line_segment_intersection(Triangle const& tri, LineSegment const& ln) {

    // Find perpendicular to triangle plane and check if triangle is degenerate triangle.
    Point3d perpend = perpendicular(tri);
    // TODO: magnitude
    if (equal_to_zero(norm(perpend), 1)) {
        return line_segment_line_segment_intersection(
            degenerate_triangle_to_line_segment(tri), ln);
    }

    // Find transition to basis where:
    // 1) center of coordinates moved to one of vetecies of triangle
    // 2) two index vectors are two sides of triangle that touch vertex choosen in prev step
    // 3) 3rd index vector is perpendicular to triangle plane
    Mat4d inv_trans = transformation_matrix(
        tri[1] - tri[0],
        tri[2] - tri[0],
        perpend,
        tri[0]);

    Mat4d trans = inv(inv_trans);

    // Transform line segment into new basis.
    Point4d a = trans * to_homogen(ln[0]);
    Point4d b = trans * to_homogen(ln[1]);

    // TODO: magnitude
    if (equal_to_zero(a[2], 1) && equal_to_zero(b[2], 1)) {
        auto x = triangle_line_segment_intersection(
            Triangle2d{
                Point2d{0,0},
                Point2d{0,0},
                Point2d{0,0}},
            LineSegment2d{
                Point2d{a[0], a[1]},
                Point2d{b[0], b[1]}});
        
        if (!x)
            return std::nullopt;

        Point3d t {x.value()[0], x.value()[1], 0};
        return from_homogen(inv_trans * to_homogen(t));
    }

    // If both ends of segment lie on one side of a triangle plane, there is no intersection.
    if (a[2] * b[2] > 0)
        return std::nullopt;

    // Find point where line segment intersects triangle plane.
    double k = abs(a[2]) / (abs(a[2]) + abs(b[2]));
    Point3d x = mid_point(
        from_homogen(a),
        from_homogen(b),
        k);

    // Check that `x` lies inside of triangle.
    // Because two sides of triangle are now basis vectors,
    // vertices of triangle has coordinates (0,0) (1,0) (0,1).
    if (x[0] >= 0 && x[1] >= 0 && x[0] + x[1] <= 1)
        return from_homogen(inv_trans * to_homogen(x));

    return std::nullopt;
}

#include <gtest/gtest.h>

TEST(TriangleLineSegmentIntersection, TrivialCase) {
    auto x = triangle_line_segment_intersection(
        Triangle{
            Point3d{1,0,0},
            Point3d{0,1,0},
            Point3d{0,0,1}},
        LineSegment{
            Point3d{0,0,0},
            Point3d{1,1,1}}
    );
    ASSERT_TRUE(x);
    EXPECT_DOUBLE_EQ(1./3, x.value()[0]);
    EXPECT_DOUBLE_EQ(1./3, x.value()[1]);
    EXPECT_DOUBLE_EQ(1./3, x.value()[2]);
}

TEST(TriangleLineSegmentIntersection, NoIntersection) {
    auto x = triangle_line_segment_intersection(
        Triangle{
            Point3d{1,0,0},
            Point3d{0,1,0},
            Point3d{0,0,1}},
        LineSegment{
            Point3d{0,0,0},
            Point3d{-1,-1,-1}}
    );
    ASSERT_FALSE(x);
}

TEST(TriangleLineSegmentIntersection, FlatOrbitraryInput0) {
    auto x = triangle_line_segment_intersection(
        Triangle{
            Point3d{ 8,-3, 0},
            Point3d{-7, 2, 0},
            Point3d{-6,-1, 0}},
        LineSegment{
            Point3d{ 5, 6,-4},
            Point3d{ 7, 9,-2}}
    );
    ASSERT_FALSE(x);
}

TEST(TriangleLineSegmentIntersection, FlatOrbitraryInput1) {
    auto x = triangle_line_segment_intersection(
        Triangle{
            Point3d{ 1, 3, 0},
            Point3d{ 7,-3, 0},
            Point3d{-1,-6, 0}},
        LineSegment{
            Point3d{ 0,-4, 1},
            Point3d{-3,-7,-9}}
    );
    ASSERT_TRUE(x);
    EXPECT_DOUBLE_EQ(-0.3, x.value()[0]);
    EXPECT_DOUBLE_EQ(-4.3, x.value()[1]);
    EXPECT_DOUBLE_EQ( 0.0, x.value()[2]);
}
