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
    const std::vector<tilt_compensation::Point3f> points {{1.0F, 2.0F, 3.0F},
                                                          {4.0F, 5.0F, 6.0F},
                                                          {7.0F, 8.0F, 9.0F} };
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