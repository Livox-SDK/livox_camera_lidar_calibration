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
#include <vector>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "common.h"
#include "result_verify.h"
#include "CustomMsg.h"

using namespace std;
using namespace cv;

void getColor(int &result_r, int &result_g, int &result_b, float cur_depth);
void loadPointcloudFromROSBag(const string& bag_path);

float max_depth = 60;
float min_depth = 3;

cv::Mat src_img;

vector<livox_ros_driver::CustomMsg> lidar_datas; 
int threshold_lidar;  // number of cloud point on the photo
string input_bag_path, input_photo_path, output_path, intrinsic_path, extrinsic_path;

void loadPointcloudFromROSBag(const string& bag_path) {
    ROS_INFO("Start to load the rosbag %s", bag_path.c_str());
    rosbag::Bag bag;
    try {
        bag.open(bag_path, rosbag::bagmode::Read);
    } catch (rosbag::BagException e) {
        ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
        return;
    }

    vector<string> types;
    types.push_back(string("livox_ros_driver/CustomMsg"));  // message title
    rosbag::View view(bag, rosbag::TypeQuery(types));

    for (const rosbag::MessageInstance& m : view) {
        livox_ros_driver::CustomMsg livoxCloud = *(m.instantiate<livox_ros_driver::CustomMsg>()); // message type
        lidar_datas.push_back(livoxCloud);
        if (lidar_datas.size() > (threshold_lidar/24000 + 1)) {
            break;
        }
    }
}

// set the color by distance to the cloud
void getColor(int &result_r, int &result_g, int &result_b, float cur_depth) {
    float scale = (max_depth - min_depth)/10;
    if (cur_depth < min_depth) {
        result_r = 0;
        result_g = 0;
        result_b = 0xff;
    }
    else if (cur_depth < min_depth + scale) {
        result_r = 0;
        result_g = int((cur_depth - min_depth) / scale * 255) & 0xff;
        result_b = 0xff;
    }
    else if (cur_depth < min_depth + scale*2) {
        result_r = 0;
        result_g = 0xff;
        result_b = (0xff - int((cur_depth - min_depth - scale) / scale * 255)) & 0xff;
    }
    else if (cur_depth < min_depth + scale*4) {
        result_r = int((cur_depth - min_depth - scale*2) / scale * 255) & 0xff;
        result_g = 0xff;
        result_b = 0;
    }
    else if (cur_depth < min_depth + scale*7) {
        result_r = 0xff;
        result_g = (0xff - int((cur_depth - min_depth - scale*4) / scale * 255)) & 0xff;
        result_b = 0;
    }
    else if (cur_depth < min_depth + scale*10) {
        result_r = 0xff;
        result_g = 0;
        result_b = int((cur_depth - min_depth - scale*7) / scale * 255) & 0xff;
    }
    else {
        result_r = 0xff;
        result_g = 0;
        result_b = 0xff;
    }

}

void getParameters() {
    cout << "Get the parameters from the launch file" << endl;

    if (!ros::param::get("input_bag_path", input_bag_path)) {
        cout << "Can not get the value of input_bag_path" << endl;
        exit(1);
    }
    if (!ros::param::get("input_photo_path", input_photo_path)) {
        cout << "Can not get the value of input_photo_path" << endl;
        exit(1);
    }
    if (!ros::param::get("output_path", output_path)) {
        cout << "Can not get the value of output_path" << endl;
        exit(1);
    }
    if (!ros::param::get("threshold_lidar", threshold_lidar)) {
        cout << "Can not get the value of threshold_lidar" << endl;
        exit(1);
    }
    if (!ros::param::get("intrinsic_path", intrinsic_path)) {
        cout << "Can not get the value of intrinsic_path" << endl;
        exit(1);
    }
    if (!ros::param::get("extrinsic_path", extrinsic_path)) {
        cout << "Can not get the value of extrinsic_path" << endl;
        exit(1);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "projectCloud");
    getParameters();

    src_img = cv::imread(input_photo_path);

    if(src_img.empty()) {  // use the file name to search the photo
        cout << "No Picture found by filename: " << input_photo_path << endl;
        return 0;
    }

    loadPointcloudFromROSBag(input_bag_path);

    vector<float> intrinsic;
    getIntrinsic(intrinsic_path, intrinsic);
    vector<float> distortion;
    getDistortion(intrinsic_path, distortion);
    vector<float> extrinsic;
    getExtrinsic(extrinsic_path, extrinsic);

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

    ROS_INFO("Start to project the lidar cloud");
    float x, y, z;
    float theoryUV[2] = {0, 0};
    int myCount = 0;
    for (unsigned int i = 0; i < lidar_datas.size(); ++i) {
        for (unsigned int j = 0; j < lidar_datas[i].point_num; ++j) {
            x = lidar_datas[i].points[j].x;
            y = lidar_datas[i].points[j].y;
            z = lidar_datas[i].points[j].z;

            getTheoreticalUV(theoryUV, intrinsic, extrinsic, x, y, z);
            int u = floor(theoryUV[0] + 0.5);
            int v = floor(theoryUV[1] + 0.5);
            int r,g,b;
            getColor(r, g, b, x);

            Point p(u, v);
            circle(src_img, p, 1, Scalar(b, g, r), -1);
            ++myCount;
            if (myCount > threshold_lidar) {
                break;
            }
        }
        if (myCount > threshold_lidar) {
            break;
        }
    }
    ROS_INFO("Show and save the picture, tap any key to close the photo");

    cv::Mat view, rview, map1, map2;
    cv::Size imageSize = src_img.size();
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
    cv::remap(src_img, src_img, map1, map2, cv::INTER_LINEAR);  // correct the distortion
    cv::namedWindow("source", cv::WINDOW_KEEPRATIO);
    
    cv::imshow("source", src_img);
    cv::waitKey(0);
    cv::imwrite(output_path, src_img);
    return 0;
}



