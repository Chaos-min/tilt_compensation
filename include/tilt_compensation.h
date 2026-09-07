#pragma once
#include <vector>

namespace tilt_compensation {

struct Point3f {
    float x;
    float y;
    float z;
};

struct TiltAngles {
    double x_degrees;
    double y_degrees;
};

void compensateInPlace(
    std::vector<Point3f>& points,
    TiltAngles sensor_reading,
    TiltAngles zero_bias);

std::vector<Point3f> compensate(
    const std::vector<Point3f>& points,
    TiltAngles sensor_reading,
    TiltAngles zero_bias);

}  // namespace tilt_compensation


