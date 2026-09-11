#include "tilt_compensation.h"

#include <cmath>
#include <cstdio>
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
    std::vector<Point3f> x_points{{0.0F, cosine, sine}};
    std::vector<Point3f> y_points{{cosine, 0.0F, sine}};

    tilt_compensation::compensateInPlace(
        x_points, TiltAngles{5.5, 0.0}, TiltAngles{0.5, 0.0});
    tilt_compensation::compensateInPlace(
        y_points, TiltAngles{0.0, 5.0}, TiltAngles{0.0, 0.0});

    assertPointNear(x_points[0], Point3f{0.0F, 1.0F, 0.0F});
    assertPointNear(y_points[0], Point3f{1.0F, 0.0F, 0.0F});
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
        {0.0F, 1.0F, 0.0F},
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
    const float up_x = 0.06975647F;
    const float up_y = -0.05233596F;
    const float up_z = std::sqrt(1.0F - up_x * up_x - up_y * up_y);
    std::vector<Point3f> points{{up_z, 0.0F, -up_x}, {0.0F, up_z, -up_y}};
    const float original_range = std::sqrt(
        points[0].x * points[0].x + points[0].z * points[0].z);

    tilt_compensation::compensateInPlace(
        points, TiltAngles{3.0, -4.0}, TiltAngles{0.0, 0.0});

    require(near(points[0].z, 0.0F), "first horizontal direction was not leveled");
    require(near(points[1].z, 0.0F), "second horizontal direction was not leveled");
    const float corrected_range =
        std::sqrt(points[0].x * points[0].x + points[0].y * points[0].y +
                  points[0].z * points[0].z);
    require(near(corrected_range, original_range), "rotation changed point range");
}

}  // namespace

int main() {
    try {
        identityWhenReadingEqualsBias();
        singleAxisTiltsLevelHorizontalPoints();
        copyApiMatchesInPlaceWithoutChangingInput();
        invalidAnglesThrowBeforeModifyingPoints();
        nonFinitePointsRemainUnchanged();
        combinedTiltLevelsPlaneAndPreservesRange();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 1;
    }
}
