#include <cassert>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "libgnss++/visibility/fisheye_camera.hpp"
#include "libgnss++/visibility/proj_fisheye.hpp"
#include "libgnss++/visibility/im2fisheye.hpp"
#include "libgnss++/visibility/mask_fisheye.hpp"
#include "libgnss++/visibility/kml_tour.hpp"
#include "libgnss++/visibility/satellite_visibility.hpp"

using namespace libgnss::visibility;

static constexpr double kEps = 1e-6;

void test_wrapTo360() {
    assert(std::abs(wrapTo360(0.0) - 0.0) < kEps);
    assert(std::abs(wrapTo360(360.0) - 0.0) < kEps);
    assert(std::abs(wrapTo360(450.0) - 90.0) < kEps);
    assert(std::abs(wrapTo360(-90.0) - 270.0) < kEps);
    assert(std::abs(wrapTo360(-360.0) - 0.0) < kEps);
    std::cout << "  PASS: wrapTo360\n";
}

void test_polyval() {
    // p(x) = 2x + 3
    std::vector<double> p = {2.0, 3.0};
    assert(std::abs(polyval(p, 0.0) - 3.0) < kEps);
    assert(std::abs(polyval(p, 1.0) - 5.0) < kEps);
    assert(std::abs(polyval(p, 10.0) - 23.0) < kEps);

    // p(x) = x^2 - 4x + 4
    std::vector<double> p2 = {1.0, -4.0, 4.0};
    assert(std::abs(polyval(p2, 2.0) - 0.0) < kEps);
    assert(std::abs(polyval(p2, 0.0) - 4.0) < kEps);
    std::cout << "  PASS: polyval\n";
}

void test_camera_default() {
    FisheyeCamera cam;
    assert(cam.width == 600);
    assert(cam.height == 600);
    assert(std::abs(cam.xc - 300.0) < kEps);
    assert(std::abs(cam.yc - 300.0) < kEps);
    std::cout << "  PASS: camera defaults\n";
}

void test_camera_polyfit() {
    // Match fishcam.xml: el=[90, 0] -> r=[0, 300], degree=1
    FisheyeCamera cam;
    cam.el_calib = {90.0, 0.0};
    cam.r_calib = {0.0, 300.0};
    cam.poly_degree = 1;

    auto poly = cam.fitPolynomial();
    assert(poly.size() == 2);

    // r = a * el + b => r(90) = 0, r(0) = 300
    // a = -300/90 = -10/3, b = 300
    double r90 = polyval(poly, 90.0);
    double r0 = polyval(poly, 0.0);
    assert(std::abs(r90 - 0.0) < 0.01);
    assert(std::abs(r0 - 300.0) < 0.01);

    // r(45) should be 150
    double r45 = polyval(poly, 45.0);
    assert(std::abs(r45 - 150.0) < 0.01);
    std::cout << "  PASS: camera polyfit\n";
}

void test_proj_fisheye_zenith() {
    // Satellite at zenith (el=90) should project to image center
    FisheyeCamera cam;
    cam.el_calib = {90.0, 0.0};
    cam.r_calib = {0.0, 300.0};
    cam.poly_degree = 1;

    auto result = projFisheye({0.0}, {90.0}, cam);
    assert(result.valid_idx.size() == 1);
    // r=0, angle = (-0+90)deg = 90deg => cos(90)=0, sin(90)=1
    // mx = 0*cos(90) + 300 = 300
    // my = 600 - (0*sin(90) + 300) = 300
    assert(std::abs(result.px[0] - 300.0) < 0.01);
    assert(std::abs(result.py[0] - 300.0) < 0.01);
    std::cout << "  PASS: proj_fisheye zenith\n";
}

void test_proj_fisheye_horizon() {
    // Satellite at horizon (el=0) should be at edge
    FisheyeCamera cam;
    cam.el_calib = {90.0, 0.0};
    cam.r_calib = {0.0, 300.0};
    cam.poly_degree = 1;

    // az=0 (North), el=0 => r=300
    // angle_rad = (-0+90) * pi/180 = pi/2
    // mx = 300*cos(pi/2) + 300 = 300
    // my = 600 - (300*sin(pi/2) + 300) = 0
    auto result = projFisheye({0.0}, {0.0}, cam);
    assert(result.valid_idx.size() == 1);
    assert(std::abs(result.px[0] - 300.0) < 0.5);
    assert(std::abs(result.py[0] - 0.0) < 0.5);
    std::cout << "  PASS: proj_fisheye horizon\n";
}

void test_mask_fisheye_identical() {
    // Identical images should produce zero mask
    cv::Mat img = cv::Mat::zeros(600, 600, CV_8UC3);
    img.setTo(cv::Scalar(128, 128, 128));
    cv::Mat mask = maskFisheye(img, img);
    assert(cv::countNonZero(mask) == 0);
    std::cout << "  PASS: mask_fisheye identical\n";
}

void test_mask_fisheye_with_obstacle() {
    // White opensky, dark obstacle in center
    cv::Mat opensky = cv::Mat(600, 600, CV_8UC3, cv::Scalar(200, 200, 200));
    cv::Mat fisheye = opensky.clone();

    // Add dark region (obstacle) in center 100x100
    cv::rectangle(fisheye, cv::Rect(250, 250, 100, 100), cv::Scalar(10, 10, 10), cv::FILLED);

    cv::Mat mask = maskFisheye(fisheye, opensky, 25, 3);

    // Center should be masked (obstacle)
    assert(mask.at<uint8_t>(300, 300) > 0);
    // Corner should be clear
    assert(mask.at<uint8_t>(0, 0) == 0);
    std::cout << "  PASS: mask_fisheye with obstacle\n";
}

void test_satellite_visibility() {
    // Create a simple mask: upper half = obstacle, lower half = clear
    cv::Mat mask = cv::Mat::zeros(600, 600, CV_8UC1);
    mask(cv::Rect(0, 0, 600, 300)).setTo(255);

    FisheyeCamera cam;
    cam.el_calib = {90.0, 0.0};
    cam.r_calib = {0.0, 300.0};
    cam.poly_degree = 1;

    // Zenith satellite (az=0, el=90) -> center (300,300) -> lower half -> LOS
    // North horizon satellite (az=0, el=0) -> (300,0) -> upper half -> NLOS
    auto vis = determineSatelliteVisibility({0.0, 0.0}, {90.0, 0.0}, mask, cam);
    assert(vis.valid_idx.size() == 2);
    assert(!vis.is_nlos[0]);  // zenith = LOS
    assert(vis.is_nlos[1]);   // horizon = NLOS
    assert(vis.num_los == 1);
    assert(vis.num_nlos == 1);
    std::cout << "  PASS: satellite visibility\n";
}

void test_kml_tour_write() {
    std::string path = "/tmp/test_visibility_tour.kml";
    KmlWaypoint wp{35.667387, 139.791677, 2.0, 0.0};
    assert(writeKmlTour(path, {wp}, 160.0));

    std::ifstream ifs(path);
    std::string content((std::istreambuf_iterator<char>(ifs)),
                        std::istreambuf_iterator<char>());
    assert(content.find("<gx:Tour>") != std::string::npos);
    assert(content.find("<tilt>180</tilt>") != std::string::npos);
    assert(content.find("<gx:horizFov>160") != std::string::npos);
    assert(content.find("35.667387") != std::string::npos);
    // heading=0 -> wrapTo360(0+180) = 180
    assert(content.find("180.0") != std::string::npos);
    std::cout << "  PASS: kml tour write\n";
}

void test_im2fisheye_basic() {
    // Create a simple 100x100 colored image
    cv::Mat input(100, 100, CV_8UC3, cv::Scalar(50, 100, 150));

    FisheyeCamera cam;
    cam.el_calib = {90.0, 0.0};
    cam.r_calib = {0.0, 300.0};
    cam.poly_degree = 1;

    cv::Mat fisheye = im2fisheye(input, 160.0, cam);
    assert(fisheye.rows == cam.height);
    assert(fisheye.cols == cam.width);
    assert(fisheye.channels() == 3);

    // At least some pixels should be non-zero (image was colored)
    cv::Mat gray;
    cv::cvtColor(fisheye, gray, cv::COLOR_BGR2GRAY);
    assert(cv::countNonZero(gray) > 0);
    std::cout << "  PASS: im2fisheye basic\n";
}

int main() {
    std::cout << "Running visibility tests...\n";
    test_wrapTo360();
    test_polyval();
    test_camera_default();
    test_camera_polyfit();
    test_proj_fisheye_zenith();
    test_proj_fisheye_horizon();
    test_mask_fisheye_identical();
    test_mask_fisheye_with_obstacle();
    test_satellite_visibility();
    test_kml_tour_write();
    test_im2fisheye_basic();
    std::cout << "All visibility tests passed!\n";
    return 0;
}
