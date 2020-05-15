#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <string>

#include "common.h"

using namespace std;

void writeData(const string filename, const float x, const float y, uint mode);

cv::Mat gray_img, src_img;
cv::RNG  random_number_generator;
string photo_path, output_name, intrinsic_path;

void writeData(const string filename, const float x, const float y, uint mode) {
    ofstream outfile(filename.c_str(), ios_base::app);
    if (!outfile) {
        cout << "Can not open the file: " << filename << endl;
        exit(0);
    }
    switch(mode) {
        case(0):
            outfile << "photo" << endl;
            outfile << "1" << endl;
            break;
        case(1):
            outfile << "2" << endl;
            break;
        case(2):
            outfile << "3" << endl;
            break;
        case(3):
            outfile << "4" << endl;
            break;
        default:
            cout << "[writeData] - Code error, unknown mode" << endl;
            exit(0);
    }
    outfile << float2str(x) << "        " << float2str(y) << endl;
}

void getParameters() {
    cout << "Get the parameters from the launch file" << endl;

    if (!ros::param::get("input_photo_path", photo_path)) {
        cout << "Can not get the value of input_photo_path" << endl;
        exit(1);
    }
    if (!ros::param::get("ouput_path", output_name)) {
        cout << "Can not get the value of ouput_path" << endl;
        exit(1);
    }
    if (!ros::param::get("intrinsic_path", intrinsic_path)) {
        cout << "Can not get the value of intrinsic_path" << endl;
        exit(1);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cornerPhoto");
    getParameters();

    src_img = cv::imread(photo_path);

    if(src_img.empty()) {  // use the file name to search the photo
        cout << "No Picture found by filename: " << photo_path << endl;
        return 0;
    }

    vector<float> intrinsic;
    getIntrinsic(intrinsic_path, intrinsic);
    vector<float> distortion;
    getDistortion(intrinsic_path, distortion);

	// set intrinsic parameters of the camera
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = intrinsic[0];
    cameraMatrix.at<double>(0, 2) = intrinsic[2];
    cameraMatrix.at<double>(1, 1) = intrinsic[4];
    cameraMatrix.at<double>(1, 2) = intrinsic[5];

	// set radial distortion and tangential distortion
    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    distCoeffs.at<double>(0, 0) = distortion[0];
    distCoeffs.at<double>(1, 0) = distortion[1];
    distCoeffs.at<double>(2, 0) = distortion[2];
    distCoeffs.at<double>(3, 0) = distortion[3];
    distCoeffs.at<double>(4, 0) = distortion[4];

    cv::Mat view, rview, map1, map2;
    cv::Size imageSize = src_img.size();
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
    cv::remap(src_img, src_img, map1, map2, cv::INTER_LINEAR);  // correct the distortion

    cout << "Please note the four corners, and then tap a random key to give the coordinate" << endl;
    // cv::namedWindow("source", CV_WINDOW_KEEPRATIO);
    cv::namedWindow("source");
    cv::imshow("source", src_img);
    cv::waitKey(0);
    
    cv::destroyWindow("source");
    vector<cv::Point2f> corners;
    cout << "Give the corner coordinate, finish by 0 0" << endl;
    while(1) {
        cv::Point2f p;
        cin >> p.x >> p.y;
        if(p.x < 0.1 && p.y < 0.1) {  // finish by typing "0 0"
            break;
        }
        corners.push_back(p);
    }
    if (!corners.size()) {
        cout << "No input corners, end process" << endl;
        return 0;
    }
    cv::Size winSize = cv::Size(5, 5);
	cv::Size zerozone = cv::Size(-1, -1);
    cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 40, 0.001);
    
    // cv::namedWindow("output", CV_WINDOW_KEEPRATIO);
    cv::namedWindow("output");
    cv::cvtColor(src_img, gray_img, cv::COLOR_BGR2GRAY);
    cv::cornerSubPix(gray_img, corners, winSize, zerozone, criteria);

    cv::Mat result_img = src_img.clone();
    for(uint t = 0; t < corners.size(); ++t) {
        cv::circle(result_img, corners[t], 3, cv::Scalar(random_number_generator.uniform(0, 255), random_number_generator.uniform(0, 255), random_number_generator.uniform(0, 255)), 1, 8, 0);
        printf("(%.3f %.3f)", corners[t].x, corners[t].y);
        writeData(output_name, corners[t].x, corners[t].y, t);
    }
    
    cout << endl << "Result saved, tap a random key to finish the process" << endl;
    cv::namedWindow("output");
    imshow("output", result_img);
    cv::waitKey(0);
    return 0;
}







