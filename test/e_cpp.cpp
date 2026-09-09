// C++调用示例
#include "tilt_compensation.h"

#include <iostream>
#include <exception>

#include <vector>

//const std::vector<tilt_compensation::Point3f> points {{1.0F, 2.0F, 3.0F}};
//const tilt_compensation::TiltAngles reading {2.0103, -0.5132};
//const tilt_compensation::TiltAngles zero_bias {0.1020, -0.0310};
//
//std::vector<tilt_compensation::Point3f> in_place = points;
//tilt_compensation::compensateInPlace(in_place, reading, zero_bias);
//
//// Algorithm output
//std::vector<tilt_compensation::Point3f> corrected = 
//    tilt_compensation::compensate(in_place, sensor_reading, zero_bias);


void test_tilt_compensation_output(
    const std::vector<tilt_compensation::Point3f>& in_place,
    const tilt_compensation::TiltAngles& reading,
    const tilt_compensation::TiltAngles& zero_bias) {
    
    std::vector<tilt_compensation::Point3f> corrected =
        tilt_compensation::compensate(in_place, reading, zero_bias);

    std::cout << "In-place: " << in_place.size() << std::endl;
    for (std::size_t i = 0; i < in_place.size(); ++i) {
        std::cout << "Point " << i << ": " << in_place[i].x << ", " << in_place[i].y << ", " << in_place[i].z << std::endl;
    }

    std::cout << "Corrected: " << corrected.size() << std::endl;
    for (std::size_t i = 0; i < corrected.size(); ++i) {
        std::cout << "Point " << i << ": " << corrected[i].x << ", " << corrected[i].y << ", " << corrected[i].z << std::endl;
    }
}

int main() {
    // Algorithm param
    const std::vector<tilt_compensation::Point3f> points {{-1.0F, -2.0F, -3.0F},
                                                          {4.0F, 5.0F, 6.0F},
                                                          {7.0F, 8.0F, 9.0F},
                                                          {10.0F, 11.0F, 12.0F} };
    const tilt_compensation::TiltAngles reading {-0.0697, 0.0523};
    const tilt_compensation::TiltAngles zero_bias {-0.0021, 0.0013};
    
    try {
        std::vector<tilt_compensation::Point3f> in_place = points;
        test_tilt_compensation_output(in_place, reading, zero_bias);
        std::cout << "Test passed" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}

/*
int main() {
    try {
        const std::vector<tilt_compensation::Point3f> points {
            {0.9961947F, 0.0F, 0.0871557F},
            {1.0F, 2.0F, 3.0F},
            {4.0F, 5.0F, 6.0F}
        };
        // points X=5.5° Y=0.0° zero_bias X=0.5° Y=0.0°  tx = 5.5° - 0.5° ty = 0.0° - 0.0°
        const tilt_compensation::TiltAngles reading {5.5, 0.0};
        const tilt_compensation::TiltAngles zero_bias {0.5, 0.0};

        const std::vector<tilt_compensation::Point3f> correct = 
            tilt_compensation::compensate(points, reading, zero_bias);

        std::cout << "Corrected: " << correct.size() << std::endl;

        for (std::size_t i = 0; i < correct.size(); ++i) {
            std::cout
                << "points [" << i << "]"
                << "raw = (" << points[i].x << ", " << points[i].y << ", " << points[i].z << ")"
                << "correct = (" << correct[i].x << ", " << correct[i].y << ", " << correct[i].z << ")\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}*/

// 度分秒(dms) 与 十进制度数(decimal)转换
/*
#include <cmath>

struct DMS {
    int deg;
    int min;
    float sec;
};

float dmsToDecimal(int deg, int min, float sec) {
    const float sign = deg < 0 ? -1.0 : 1.0;
    return sign * (std::fabs(static_cast<float>(deg)) + min / 60.0 + sec / 3600.0);
}

DMS decimalToDMS(float deg) {
    const bool negative = std::signbit(deg);
    const int totalSeconds = static_cast<int>(std::lround(std::fabs(deg) * 3600.0));

    DMS out;
    out.deg = static_cast<int>(totalSeconds / 3600);
    out.min = static_cast<int>((totalSeconds % 3600) / 60);
    out.sec = static_cast<float>(totalSeconds % 60);

    if (negative) out.deg = -out.deg;
    return out;
}

int main() {

    float decimal = 123.45678;
    DMS dms = decimalToDMS(decimal);
    std::cout << "Decimal: " << decimal << " -> DMS: " << dms.deg << "°" << dms.min << "'" << dms.sec << "\"";

    decimal  = dmsToDecimal(dms.deg, dms.min, dms.sec);
    std::cout << "DMS: " << dms.deg << "°" << dms.min << "'" << dms.sec << "\" -> Decimal: " << decimal << std::endl;
}*/
