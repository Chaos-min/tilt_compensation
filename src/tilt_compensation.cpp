#include "tilt_compensation.h"

#include <cmath>
#include <stdexcept>

namespace tilt_compensation {
namespace {

const double kDegreesToRadians = 3.14159265358979323846 / 180.0;
const double kPhysicalTolerance = 1.0e-12;

struct Matrix3 {
    double value[3][3];
};

bool validAngle(double angle) {
    return std::isfinite(angle) && std::fabs(angle) <= 90.0;
}

Matrix3 buildLevelingMatrix(TiltAngles reading, TiltAngles zero_bias) {
    if (!validAngle(reading.x_degrees) || !validAngle(reading.y_degrees) ||
        !validAngle(zero_bias.x_degrees) || !validAngle(zero_bias.y_degrees)) {
        throw std::invalid_argument(
            "tilt angles must be finite and within [-90, 90] degrees");
    }

    const double tx_degrees = reading.x_degrees - zero_bias.x_degrees;
    const double ty_degrees = reading.y_degrees - zero_bias.y_degrees;
    if (!validAngle(tx_degrees) || !validAngle(ty_degrees)) {
        throw std::invalid_argument(
            "calibrated tilt must be within [-90, 90] degrees");
    }

    const double tx = tx_degrees * kDegreesToRadians;
    const double ty = ty_degrees * kDegreesToRadians;
    const double sine_x = std::sin(tx);
    const double sine_y = std::sin(ty);
    double horizontal_squared = sine_x * sine_x + sine_y * sine_y;
    if (horizontal_squared > 1.0 + kPhysicalTolerance) {
        throw std::invalid_argument("X/Y tilt combination is not physically valid");
    }
    if (horizontal_squared > 1.0) {
        horizontal_squared = 1.0;
    }

    const double up_x = -sine_x;
    const double up_y = sine_y;
    const double up_z = std::sqrt(1.0 - horizontal_squared);
    const double sine = std::sqrt(horizontal_squared);

    Matrix3 matrix = {{{1.0, 0.0, 0.0},
                       {0.0, 1.0, 0.0},
                       {0.0, 0.0, 1.0}}};
    if (sine == 0.0) {
        return matrix;
    }

    const double axis_x = up_y / sine;
    const double axis_y = -up_x / sine;
    const double one_minus_cosine = horizontal_squared / (1.0 + up_z);

    matrix.value[0][0] = up_z + axis_x * axis_x * one_minus_cosine;
    matrix.value[0][1] = axis_x * axis_y * one_minus_cosine;
    matrix.value[0][2] = axis_y * sine;
    matrix.value[1][0] = matrix.value[0][1];
    matrix.value[1][1] = up_z + axis_y * axis_y * one_minus_cosine;
    matrix.value[1][2] = -axis_x * sine;
    matrix.value[2][0] = -axis_y * sine;
    matrix.value[2][1] = axis_x * sine;
    matrix.value[2][2] = up_z;
    return matrix;
}

bool finitePoint(const Point3f& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           std::isfinite(point.z);
}

}  // namespace

void compensateInPlace(
    std::vector<Point3f>& points,
    TiltAngles sensor_reading,
    TiltAngles zero_bias) {
    const Matrix3 matrix = buildLevelingMatrix(sensor_reading, zero_bias);

    for (std::vector<Point3f>::iterator it = points.begin(); it != points.end();
         ++it) {
        if (!finitePoint(*it)) {
            continue;
        }

        const double x = it->x;
        const double y = it->y;
        const double z = it->z;
        it->x = static_cast<float>(
            matrix.value[0][0] * x + matrix.value[0][1] * y + matrix.value[0][2] * z);
        it->y = static_cast<float>(
            matrix.value[1][0] * x + matrix.value[1][1] * y + matrix.value[1][2] * z);
        it->z = static_cast<float>(
            matrix.value[2][0] * x + matrix.value[2][1] * y + matrix.value[2][2] * z);
    }
}

std::vector<Point3f> compensate(
    const std::vector<Point3f>& points,
    TiltAngles sensor_reading,
    TiltAngles zero_bias) {
    std::vector<Point3f> corrected = points;
    compensateInPlace(corrected, sensor_reading, zero_bias);
    return corrected;
}

}  // namespace tilt_compensation
