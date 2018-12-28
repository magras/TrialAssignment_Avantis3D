#ifndef EQUAL_TO_ZERO_H
#define EQUAL_TO_ZERO_H

// This two methods should be used when catastrophic cancelation happens.
// I'm using `magnitude` parameter to specify the order of magnitude of
// numbers before subtraction.

inline bool equal_to_zero(double x, double magnitude) {
    double epsilon = std::numeric_limits<double>::epsilon();
    return std::abs(x) <= epsilon * magnitude;
}

inline bool not_equal_to_zero(double x, double magnitude) {
    return !equal_to_zero(x, magnitude);
}

#endif // EQUAL_TO_ZERO_H
