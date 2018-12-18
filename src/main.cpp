#include <array>
#include <iostream>
#include <optional>
#include <blaze/math/StaticMatrix.h>
#include <blaze/math/StaticVector.h>

using Mat4d = blaze::StaticMatrix<double, 4, 4>;
using Point3d = blaze::StaticVector<double, 3>;
using Point4d = blaze::StaticVector<double, 4>;
using Triangle = std::array<Point3d, 3>;
using LineSegment = std::array<Point3d, 2>;

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

std::optional<Point3d> triangle_line_segment_cross(Triangle const& tri, LineSegment const& ln) {

    // Find transition to basis where:
    // 1) center of coordinates moved to one of vetecies of triangle
    // 2) two index vectors are two sides of triangle that touch vertex choosen in prev step
    // 3) 3rd index vector is perpendicular to triangle plane
    Mat4d inv_trans = transformation_matrix(
        tri[1] - tri[0],
        tri[2] - tri[0],
        perpendicular(tri),
        tri[0]);

    Mat4d trans = inv(inv_trans);

    // Transform line segment into new basis.
    Point4d a = trans * to_homogen(ln[0]);
    Point4d b = trans * to_homogen(ln[1]);

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

TEST(TriangleLineSegmentCross, TrivialCase) {
    auto x = triangle_line_segment_cross(
        Triangle{
            Point3d{1,0,0},
            Point3d{0,1,0},
            Point3d{0,0,1}},
        LineSegment{
            Point3d{0,0,0},
            Point3d{1,1,1}}
    );
    ASSERT_TRUE(x);
    ASSERT_DOUBLE_EQ(1./3, x.value()[0]);
    ASSERT_DOUBLE_EQ(1./3, x.value()[1]);
    ASSERT_DOUBLE_EQ(1./3, x.value()[2]);
}

TEST(TriangleLineSegmentCross, NoIntersection) {
    auto x = triangle_line_segment_cross(
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

TEST(TriangleLineSegmentCross, FlatOrbitraryInput0) {
    auto x = triangle_line_segment_cross(
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

TEST(TriangleLineSegmentCross, FlatOrbitraryInput1) {
    auto x = triangle_line_segment_cross(
        Triangle{
            Point3d{ 1, 3, 0},
            Point3d{ 7,-3, 0},
            Point3d{-1,-6, 0}},
        LineSegment{
            Point3d{ 0,-4, 1},
            Point3d{-3,-7,-9}}
    );
    ASSERT_TRUE(x);
    ASSERT_DOUBLE_EQ(-0.3, x.value()[0]);
    ASSERT_DOUBLE_EQ(-4.3, x.value()[1]);
    ASSERT_DOUBLE_EQ( 0.0, x.value()[2]);
}
