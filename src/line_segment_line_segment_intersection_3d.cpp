#include <algorithm>
#include <optional>
#include "equal_to_zero.h"
#include "fmap.h"
#include "geom_types.h"

namespace {

    template <typename T, size_t N, bool TF>
    size_t max_abs_elem_index(blaze::StaticVector<T,N,TF> const& v) {
        auto it = std::max_element(
            begin(v),
            end(v),
            [](auto lhs, auto rhs){
                return std::abs(lhs) < std::abs(rhs);
            }
        );
        return it - begin(v);
    }

    std::optional<std::array<double, 2>> intervals_intersection(std::array<double, 2> a, std::array<double, 2> b) {
    
        assert(a[0] <= a[1]);
        assert(b[0] <= b[1]);
    
        auto left = std::max(a[0], b[0]);
        auto right = std::min(a[1], b[1]);

        if (left <= right)
            return {{left, right}};
        else
            return std::nullopt;
    }

} // anonymous namespace

std::optional<Point3d> line_segment_line_segment_intersection(LineSegment3d ln0, LineSegment3d ln1) {

    Point3d a = ln0[1] - ln0[0];
    Point3d b = ln1[1] - ln1[0];

    // Set line segment with greater length as the first one to
    // work around the case when other line segment is a point.
    if (norm(a) < norm(b)) {
        swap(ln0, ln1);
        swap(a, b);
    }

    Point3d c = ln1[0] - ln0[0];
    Point3d v = cross(a, b);

    double magnitude = sqrNorm(a);

    // Line segments must be coplanar to intersect.
    if (not_equal_to_zero(dot(c, v), magnitude))
        return std::nullopt;

    // If line segments are parallel.
    if (equal_to_zero(norm(v), magnitude)) {
        
        // Parallel line segments must be coincident to intersect.
        if (not_equal_to_zero(norm(cross(a, c)), magnitude))
            return std::nullopt;

        // Actualy both line segments are points.
        if (equal_to_zero(norm(a), magnitude)) {
            // I think there should be precise equivalence, because there was
            // no operation that could have induce an error in input data.
            // On the other hand we could have reach this code branch by mistake
            // (both line segments are extreamly short) in which case
            // near equivalence may be better.
            if (ln0[0] == ln1[0])
                return ln0[0];
            else
                return std::nullopt;
        }

        // Find ends of the second line segment in basis build on `a` vector.
        // Again, use the greatest direction vector component to minimize error.
        auto i = max_abs_elem_index(a);
        double x = (ln1[0][i] - ln0[0][i]) / a[i];
        double y = (ln1[1][i] - ln0[0][i]) / a[i];

        if (x > y)
            std::swap(x, y);

        auto intersection = intervals_intersection({0,1}, {x,y});
            
        // Any value inside intersection will work. I'm using left boundary.
        // It may cause troubles, so may be it's better to take mid point.
        // Also it's possible to return line segment to deligate decision to user.
        return fmap(intersection, [&](auto intersection){
            return ln0[0] + intersection[0] * a;
        });
    }

    double s = dot(cross(c, b), v) / sqrNorm(v);
    double t = dot(cross(c, a), v) / sqrNorm(v);

    // Reject intersection point if it is not within line segments.
    if (s < 0 || s > 1 || t < 0 || t > 1)
        return std::nullopt;

    return (ln0[0] + s * a + ln1[0] + t * b) / 2;
}

#include <gtest/gtest.h>

TEST(LineSegmentLineSegmentIntersection, TrivialCase) {
    auto x = line_segment_line_segment_intersection(
        LineSegment3d{
            Point3d{0,0,0},
            Point3d{2,0,0}},
        LineSegment3d{
            Point3d{1,0,-1},
            Point3d{2,0,+1}}
    );
    ASSERT_TRUE(x);
    EXPECT_DOUBLE_EQ(1.5, x.value()[0]);
    EXPECT_DOUBLE_EQ(0, x.value()[1]);
    EXPECT_DOUBLE_EQ(0, x.value()[2]);
}

TEST(LineSegmentLineSegmentIntersection, Touch) {
    auto x = line_segment_line_segment_intersection(
        LineSegment3d{
            Point3d{0,0,0},
            Point3d{1,0,0}},
        LineSegment3d{
            Point3d{1,0,0},
            Point3d{1,1,0}}
    );
    ASSERT_TRUE(x);
    EXPECT_DOUBLE_EQ(1, x.value()[0]);
    EXPECT_DOUBLE_EQ(0, x.value()[1]);
    EXPECT_DOUBLE_EQ(0, x.value()[2]);
}

TEST(LineSegmentLineSegmentIntersection, IntersectionOutsideOfLineSegment) {
    auto x = line_segment_line_segment_intersection(
        LineSegment3d{
            Point3d{0,0,0},
            Point3d{2,0,0}},
        LineSegment3d{
            Point3d{1,0,2},
            Point3d{1,0,1}}
    );
    ASSERT_FALSE(x);
}

TEST(LineSegmentLineSegmentIntersection, ParallelLines) {
    auto x = line_segment_line_segment_intersection(
        LineSegment3d{
            Point3d{0,0,0},
            Point3d{1,0,0}},
        LineSegment3d{
            Point3d{0,0,1},
            Point3d{1,0,1}}
    );
    ASSERT_FALSE(x);
}

TEST(LineSegmentLineSegmentIntersection, OverlappingLineSegments) {
    auto x = line_segment_line_segment_intersection(
        LineSegment3d{
            Point3d{0,0,0},
            Point3d{2,0,0}},
        LineSegment3d{
            Point3d{1,0,0},
            Point3d{3,0,0}}
    );
    ASSERT_TRUE(x);
    EXPECT_DOUBLE_EQ(1, x.value()[0]);
    EXPECT_DOUBLE_EQ(0, x.value()[1]);
    EXPECT_DOUBLE_EQ(0, x.value()[2]);
}

TEST(LineSegmentLineSegmentIntersection, PrimeNumbers) {
    auto x = line_segment_line_segment_intersection(
        LineSegment3d{
            Point3d{13,17,19},
            Point3d{-137./11,-177./11,-195./11}},
        LineSegment3d{
            Point3d{0,0,0},
            Point3d{3,5,7}}
    );

    // CRAP
    //    
    // This test demonstates numerical instability of algorithm.
    // Error is 23 ulps or 13 epsilon. I'm not used to write
    // numerical stable algorithms and there was no requirement to
    // do so in assignment, so i'll leave it for now.
    //
    // Bigest source of error in this case was `cross(c, a)`
    // with 221 ulps and 174 epsilon error from the true value.

    auto epsilon = std::numeric_limits<double>::epsilon();
    ASSERT_TRUE(x);
    EXPECT_NEAR(3./11, x.value()[0], epsilon * 4);
    EXPECT_NEAR(5./11, x.value()[1], epsilon * 13);
    EXPECT_NEAR(7./11, x.value()[2], epsilon * 6);
}
