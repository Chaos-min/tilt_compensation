#include "tilt_compensation.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {

using tilt_compensation::Point3f;
using tilt_compensation::TiltAngles;

bool near(float actual, float expected, float tolerance = 1.0e-5F) {
    return std::fabs(actual - expected) <= tolerance;
}

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void assertPointNear(const Point3f& actual, const Point3f& expected) {
    require(near(actual.x, expected.x), "unexpected X coordinate");
    require(near(actual.y, expected.y), "unexpected Y coordinate");
    require(near(actual.z, expected.z), "unexpected Z coordinate");
}

void identityWhenReadingEqualsBias() {
    const std::vector<Point3f> original{{1.0F, 2.0F, 3.0F}, {-4.0F, 5.0F, -6.0F}};
    std::vector<Point3f> points = original;
    std::vector<Point3f> empty;
    const TiltAngles reading{2.0103, -0.5132};

    tilt_compensation::compensateInPlace(points, reading, reading);
    tilt_compensation::compensateInPlace(empty, reading, reading);

    require(empty.empty(), "empty cloud changed");
    assertPointNear(points[0], original[0]);
    assertPointNear(points[1], original[1]);
}

void singleAxisTiltsLevelHorizontalPoints() {
    const float cosine = 0.9961947F;
    const float sine = 0.0871557F;
    std::vector<Point3f> x_points{{cosine, 0.0F, sine}};
    std::vector<Point3f> y_points{{0.0F, cosine, -sine}};

    tilt_compensation::compensateInPlace(
        x_points, TiltAngles{5.5, 0.0}, TiltAngles{0.5, 0.0});
    tilt_compensation::compensateInPlace(
        y_points, TiltAngles{0.0, 5.0}, TiltAngles{0.0, 0.0});

    assertPointNear(x_points[0], Point3f{1.0F, 0.0F, 0.0F});
    assertPointNear(y_points[0], Point3f{0.0F, 1.0F, 0.0F});
}

void copyApiMatchesInPlaceWithoutChangingInput() {
    const std::vector<Point3f> original{{1.0F, 2.0F, 3.0F}, {-4.0F, 5.0F, -6.0F}};
    std::vector<Point3f> in_place = original;
    const TiltAngles reading{3.0, -4.0};
    const TiltAngles zero_bias{0.1, -0.2};

    tilt_compensation::compensateInPlace(in_place, reading, zero_bias);
    const std::vector<Point3f> copied =
        tilt_compensation::compensate(original, reading, zero_bias);

    assertPointNear(copied[0], in_place[0]);
    assertPointNear(copied[1], in_place[1]);
    assertPointNear(original[0], Point3f{1.0F, 2.0F, 3.0F});
    assertPointNear(original[1], Point3f{-4.0F, 5.0F, -6.0F});
}

void invalidAnglesThrowBeforeModifyingPoints() {
    const std::vector<Point3f> original{{1.0F, 2.0F, 3.0F}};
    std::vector<Point3f> points = original;
    bool threw = false;

    try {
        tilt_compensation::compensateInPlace(
            points, TiltAngles{60.0, 60.0}, TiltAngles{0.0, 0.0});
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    require(threw, "physically invalid tilt did not throw");
    assertPointNear(points[0], original[0]);

    threw = false;
    try {
        tilt_compensation::compensate(
            original,
            TiltAngles{std::numeric_limits<double>::quiet_NaN(), 0.0},
            TiltAngles{0.0, 0.0});
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    require(threw, "NaN tilt did not throw");

    threw = false;
    try {
        tilt_compensation::compensate(
            original, TiltAngles{91.0, 0.0}, TiltAngles{0.0, 0.0});
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    require(threw, "out-of-range tilt did not throw");
}

void nonFinitePointsRemainUnchanged() {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    std::vector<Point3f> points{
        {1.0F, 0.0F, 0.0F},
        {nan, 2.0F, 3.0F},
        {4.0F, infinity, 6.0F}};

    tilt_compensation::compensateInPlace(
        points, TiltAngles{5.0, 0.0}, TiltAngles{0.0, 0.0});

    require(!near(points[0].z, 0.0F), "finite point was not compensated");
    require(std::isnan(points[1].x), "NaN coordinate changed");
    require(points[1].y == 2.0F && points[1].z == 3.0F,
            "point containing NaN was partially changed");
    require(points[2].x == 4.0F && std::isinf(points[2].y) && points[2].z == 6.0F,
            "point containing infinity was changed");
}

void combinedTiltLevelsPlaneAndPreservesRange() {
    const Point3f rotation_axis{-0.06975647F, 0.05233596F, 0.0F};
    const Point3f rotated_direction{-0.05213657F, -0.06949072F, -0.007605018F};
    const float original_range = std::sqrt(
        rotated_direction.x * rotated_direction.x +
        rotated_direction.y * rotated_direction.y +
        rotated_direction.z * rotated_direction.z);
    std::vector<Point3f> points{rotation_axis, rotated_direction};

    tilt_compensation::compensateInPlace(
        points, TiltAngles{3.0, -4.0}, TiltAngles{0.0, 0.0});

    assertPointNear(points[0], rotation_axis);
    assertPointNear(points[1], Point3f{-0.05233596F, -0.06975647F, 0.0F});
    const float corrected_range =
        std::sqrt(points[1].x * points[1].x + points[1].y * points[1].y +
                  points[1].z * points[1].z);
    require(near(corrected_range, original_range), "rotation changed point range");
}

}  // namespace

int main() {
    identityWhenReadingEqualsBias();
    singleAxisTiltsLevelHorizontalPoints();
    copyApiMatchesInPlaceWithoutChangingInput();
    invalidAnglesThrowBeforeModifyingPoints();
    nonFinitePointsRemainUnchanged();
    combinedTiltLevelsPlaneAndPreservesRange();
}
