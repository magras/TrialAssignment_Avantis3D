#include <optional>
#include "types.h"

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
