#include <optional>
#include "types.h"

// Forward two dementional case to more general three dimentional implementation.

// I'm loosing some performance here, but I'm lazy right now. And there was no any
// performance requirements in assignment.

// Actualy algorithm for plane would be very similar to 3d case. Essentually just
// replace cross product with perp dot product and skip coplanar check.

namespace {

    Point2d point_to_2d(Point3d const& x) {
        assert(x[2] == 0);
        return Point2d{ x[0], x[1] };
    }

    Point3d point_to_3d(Point2d const& x) {
        return Point3d{ x[0], x[1], 0 };
    }

    LineSegment line_segment_to_3d(LineSegment2d const& ls) {
        return LineSegment{
            point_to_3d(ls[0]),
            point_to_3d(ls[1])
        };
    }

} // anonymous namespace

std::optional<Point3d> line_segment_line_segment_intersection(LineSegment ln0, LineSegment ln1);

std::optional<Point2d> line_segment_line_segment_intersection(LineSegment2d const& ls0, LineSegment2d const& ls1) {
    auto x = line_segment_line_segment_intersection(
        line_segment_to_3d(ls0),
        line_segment_to_3d(ls1));

    if (!x)
        return std::nullopt;

    return point_to_2d(*x);
}

#include <gtest/gtest.h>

TEST(LineSegmentLineSegmentIntersection2d, TrivialCase) {
    auto x = line_segment_line_segment_intersection(
        LineSegment2d{
            Point2d{0,0},
            Point2d{1,1}},
        LineSegment2d{
            Point2d{0,1},
            Point2d{1,0}}
    );
    ASSERT_TRUE(x);
    EXPECT_DOUBLE_EQ(0.5, x.value()[0]);
    EXPECT_DOUBLE_EQ(0.5, x.value()[1]);
}

TEST(LineSegmentLineSegmentIntersection2d, ParallelLines) {
    auto x = line_segment_line_segment_intersection(
        LineSegment2d{
            Point2d{0,0},
            Point2d{1,0}},
        LineSegment2d{
            Point2d{0,1},
            Point2d{1,1}}
    );
    ASSERT_FALSE(x);
}

TEST(LineSegmentLineSegmentIntersection2d, CoincidentWithoutOverlap) {
    auto x = line_segment_line_segment_intersection(
        LineSegment2d{
            Point2d{0,0},
            Point2d{1,0}},
        LineSegment2d{
            Point2d{2,0},
            Point2d{3,0}}
    );
    ASSERT_FALSE(x);
}

TEST(LineSegmentLineSegmentIntersection2d, Overlapping) {
    auto x = line_segment_line_segment_intersection(
        LineSegment2d{
            Point2d{0,0},
            Point2d{2,2}},
        LineSegment2d{
            Point2d{1,1},
            Point2d{3,3}}
    );
    ASSERT_TRUE(x);
    EXPECT_DOUBLE_EQ(1, x.value()[0]);
    EXPECT_DOUBLE_EQ(1, x.value()[1]);
}

TEST(LineSegmentLineSegmentIntersection2d, PrimeNumbers) {
    auto x = line_segment_line_segment_intersection(
        LineSegment2d{
            Point2d{0,0},
            Point2d{173,179}},
        LineSegment2d{
            Point2d{83,17},
            Point2d{19,89}}
    );
    ASSERT_TRUE(x);
    EXPECT_DOUBLE_EQ(152759./2989, x.value()[0]); // 51,107059217129474740715958514553
    EXPECT_DOUBLE_EQ(158057./2989, x.value()[1]); // 52,879558380729340916694546671127
}
