#include <optional>
#include "fmap.h"
#include "geom_types.h"
#include "geom_type_transform.h"

// Forward two dementional case to more general three dimentional implementation.

// I'm loosing some performance here, but I'm lazy right now. And there was no any
// performance requirements in assignment.

// Actualy algorithm for plane would be very similar to 3d case. Essentually just
// replace cross product with perp dot product and skip coplanar check.

std::optional<Point3d> line_segment_line_segment_intersection(LineSegment3d ln0, LineSegment3d ln1);

std::optional<Point2d> line_segment_line_segment_intersection(LineSegment2d const& ls0, LineSegment2d const& ls1) {
    auto x = line_segment_line_segment_intersection(
        transform(ls0, point_to_3d),
        transform(ls1, point_to_3d));

    return fmap(x, [](auto x){
        return point_to_2d(x);
    });
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
