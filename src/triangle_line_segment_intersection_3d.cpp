#include <optional>
#include <range/v3/algorithm/all_of.hpp>
#include "equal_to_zero.h"
#include "fmap.h"
#include "geom_types.h"
#include "geom_type_transform.h"

// There is proposal P0627 about [[unreachable]] attribute.
// Actually `assert(false)` has different meaning, but it's
// platform independent and "good enough" for prototype code.
#define UNREACHABLE assert(false)

std::optional<Point3d> line_segment_line_segment_intersection(LineSegment3d ln0, LineSegment3d ln1);
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

    Point3d perpendicular(Triangle3d const& t) {
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

    LineSegment3d find_max_edge(Triangle3d const& t) {

        auto a = sqrNorm(t[0] - t[1]);
        auto b = sqrNorm(t[1] - t[2]);
        auto c = sqrNorm(t[2] - t[0]);

        if (a >= b && a >= c)
            return {t[0], t[1]};
        if (b >= a && b >= c)
            return {t[1], t[2]};
        if (c >= a && c >= b)
            return {t[2], t[0]};

        UNREACHABLE;
    }

    double sqrNorm(LineSegment3d const& ls) {
        return sqrNorm(ls[0] - ls[1]);
    }

} // anonymous namespace

std::optional<Point3d> triangle_line_segment_intersection(Triangle3d const& tri, LineSegment3d const& ln) {

    // Find perpendicular to triangle plane and check if triangle is degenerate triangle.
    Point3d perpend = perpendicular(tri);
    LineSegment3d max_edge = find_max_edge(tri);
    double magnitude = sqrNorm(max_edge);
    if (equal_to_zero(norm(perpend), magnitude)) {
        return line_segment_line_segment_intersection(max_edge, ln);
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
    LineSegment3d trans_ln = transform(ln, [&](auto p){
        return from_homogen(trans * to_homogen(p));
    });

    bool coplanar = ranges::all_of(trans_ln, [&](Point3d const& p) {
        return equal_to_zero(p[2], magnitude);
    });

    if (coplanar) {
        auto x = triangle_line_segment_intersection(
            Triangle2d{
                Point2d{0,0},
                Point2d{0,1},
                Point2d{1,0}},
            transform(trans_ln, point_to_2d));
        
        return fmap(x, [&](auto x){
            return from_homogen(inv_trans * to_homogen(point_to_3d(x)));
        });
    }

    auto a = trans_ln[0];
    auto b = trans_ln[1];

    // If both ends of segment lie on one side of a triangle plane, there is no intersection.
    if (a[2] * b[2] > 0)
        return std::nullopt;

    // Find point where line segment intersects triangle plane.
    double k = abs(a[2]) / (abs(a[2]) + abs(b[2]));
    Point3d x = mid_point(a, b, k);

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
        Triangle3d{
            Point3d{1,0,0},
            Point3d{0,1,0},
            Point3d{0,0,1}},
        LineSegment3d{
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
        Triangle3d{
            Point3d{1,0,0},
            Point3d{0,1,0},
            Point3d{0,0,1}},
        LineSegment3d{
            Point3d{0,0,0},
            Point3d{-1,-1,-1}}
    );
    ASSERT_FALSE(x);
}

TEST(TriangleLineSegmentIntersection, FlatOrbitraryInput0) {
    auto x = triangle_line_segment_intersection(
        Triangle3d{
            Point3d{ 8,-3, 0},
            Point3d{-7, 2, 0},
            Point3d{-6,-1, 0}},
        LineSegment3d{
            Point3d{ 5, 6,-4},
            Point3d{ 7, 9,-2}}
    );
    ASSERT_FALSE(x);
}

TEST(TriangleLineSegmentIntersection, FlatOrbitraryInput1) {
    auto x = triangle_line_segment_intersection(
        Triangle3d{
            Point3d{ 1, 3, 0},
            Point3d{ 7,-3, 0},
            Point3d{-1,-6, 0}},
        LineSegment3d{
            Point3d{ 0,-4, 1},
            Point3d{-3,-7,-9}}
    );
    ASSERT_TRUE(x);
    EXPECT_DOUBLE_EQ(-0.3, x.value()[0]);
    EXPECT_DOUBLE_EQ(-4.3, x.value()[1]);
    EXPECT_DOUBLE_EQ( 0.0, x.value()[2]);
}
