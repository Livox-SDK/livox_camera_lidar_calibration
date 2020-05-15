#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <stdio.h>
#include <cmath>
#include <hash_map>
#include <ctime>

#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>

#include "common.h"
#include "CustomMsg.h"

using namespace std;

void getUV(const cv::Mat &matrinxIn, const cv::Mat &matrix_out, float x, float y, float z, float* UV);
void getColor(const cv::Mat &matrinxIn, const cv::Mat &matrix_out, float x, float y, float z, int row, int col, const vector<vector<int>> &color_vector, int* RGB);
void loadPointcloudFromROSBag(const string& bag_path);

typedef pcl::PointXYZRGB PointType;
vector<livox_ros_driver::CustomMsg> lidar_datas; 
int threshold_lidar;
string input_photo_path, input_bag_path, intrinsic_path, extrinsic_path;

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
        if (lidar_datas.size() > threshold_lidar) {
            break;
        }
    }
}

// use extrinsic and intrinsic to get the corresponding U and V
void getUV(const cv::Mat &matrix_in, const cv::Mat &matrix_out, float x, float y, float z, float* UV) {
    double matrix3[4][1] = {x, y, z, 1};
    cv::Mat coordinate(4, 1, CV_64F, matrix3);
    
    // calculate the result of u and v
    cv::Mat result = matrix_in*matrix_out*coordinate;
    float u = result.at<double>(0, 0);
    float v = result.at<double>(1, 0);
    float depth = result.at<double>(2, 0);
    
    UV[0] = u / depth;
    UV[1] = v / depth;

}

// get RGB value of the lidar point
void getColor(const cv::Mat &matrix_in, const cv::Mat &matrix_out, float x, float y, float z, int row, int col, const vector<vector<int>> &color_vector, int* RGB) {
    float UV[2] = {0, 0}; 
    getUV(matrix_in, matrix_out, x, y, z, UV);  // get U and V from the x,y,z
    
    int u = int(UV[0]);
    int v = int(UV[1]);

    int32_t index = v*col + u;
    if (index < row*col && index >= 0) {
        RGB[0] = color_vector[index][0];
        RGB[1] = color_vector[index][1];
        RGB[2] = color_vector[index][2];
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
    if (!ros::param::get("threshold_lidar", threshold_lidar)) {
        cout << "Can not get the value of threshold_lidar" << endl;
        exit(1);
    }
    if (!ros::param::get("intrinsic_path", intrinsic_path)) {
        cout << "Can not get the value of intrinsic_path" << endl;
        exit(1);
    }
    if (!ros::param::get("extrinsic_path", extrinsic_path)) {
        cout << "Can not get the value o以下程序节点中如果想修改launch文件，需要到src/calibration/launch文件夹中找对应的launch文件。f extrinsic_path" << endl;
        exit(1);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "colorLidar");
    ros::NodeHandle n;
    getParameters();

    cv::Mat src_img = cv::imread(input_photo_path);
    
    if(src_img.empty()) {  // use the file name to search the photo
        cout << "No Picture found by photo_path: " << input_photo_path << endl;
        return 0;
    }

    vector<float> intrinsic;
    getIntrinsic(intrinsic_path, intrinsic);
    vector<float> distortion;
    getDistortion(intrinsic_path, distortion);
    vector<float> extrinsic;
    getExtrinsic(extrinsic_path, extrinsic);
    
    // set the intrinsic and extrinsic matrix
    double matrix1[3][3] = {{intrinsic[0], intrinsic[1], intrinsic[2]}, {intrinsic[3], intrinsic[4], intrinsic[5]}, {intrinsic[6], intrinsic[7], intrinsic[8]}}; 
    double matrix2[3][4] = {{extrinsic[0], extrinsic[1], extrinsic[2], extrinsic[3]}, {extrinsic[4], extrinsic[5], extrinsic[6], extrinsic[7]}, {extrinsic[8], extrinsic[9], extrinsic[10], extrinsic[11]}};
    
    // transform into the opencv matrix
    cv::Mat matrix_in(3, 3, CV_64F, matrix1);
    cv::Mat matrix_out(3, 4, CV_64F, matrix2);

	// set intrinsic parameters of the camera
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix.at<double>(0, 0) = intrinsic[0];
    camera_matrix.at<double>(0, 2) = intrinsic[2];
    camera_matrix.at<double>(1, 1) = intrinsic[4];
    camera_matrix.at<double>(1, 2) = intrinsic[5];

	// set radial distortion and tangential distortion
    cv::Mat distortion_coef = cv::Mat::zeros(5, 1, CV_64F);
    distortion_coef.at<double>(0, 0) = distortion[0];
    distortion_coef.at<double>(1, 0) = distortion[1];
    distortion_coef.at<double>(2, 0) = distortion[2];
    distortion_coef.at<double>(3, 0) = distortion[3];
    distortion_coef.at<double>(4, 0) = distortion[4];

    // use intrinsic matrix and distortion matrix to correct the photo first
    cv::Mat view, rview, map1, map2;
    cv::Size imageSize = src_img.size();
    cv::initUndistortRectifyMap(camera_matrix, distortion_coef, cv::Mat(),cv::getOptimalNewCameraMatrix(camera_matrix, distortion_coef, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
    cv::remap(src_img, src_img, map1, map2, cv::INTER_LINEAR);  // correct the distortion

    int row = src_img.rows;
    int col = src_img.cols;
    // cout << row << endl;
    // cout << col << endl << endl;
    vector<vector<int>> color_vector;
    color_vector.resize(row*col);
    for (unsigned int i = 0; i < color_vector.size(); ++i) {
        color_vector[i].resize(3);
    }
    
    // read photo and get all RGB information into color_vector
    ROS_INFO("Start to read the photo ");
    for (int v = 0; v < row; ++v) {
        for (int u = 0; u < col; ++u) {
            // for .bmp photo, the 3 channels are BGR
            color_vector[v*col + u][0] = src_img.at<cv::Vec3b>(v, u)[2];
            color_vector[v*col + u][1] = src_img.at<cv::Vec3b>(v, u)[1];
            color_vector[v*col + u][2] = src_img.at<cv::Vec3b>(v, u)[0];
        }
    }
    ROS_INFO("Finish saving the data ");
    
    loadPointcloudFromROSBag(input_bag_path);

    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("color_lidar", 10);
    ros::Rate loop_rate(20); // frequence 20 Hz
    
    ROS_INFO("Start to publish the point cloud");
    uint64_t num = 0;
    while(n.ok()) {
        ros::spinOnce();
        
        if(num < lidar_datas.size()) {
            pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
            cloud->is_dense = false;
            cloud->height = 1;
            cloud->width = lidar_datas[num].point_num; // get the point number of lidar data
            cloud->points.resize(cloud->width);
            for(uint64_t i = 0; i < cloud->points.size() && n.ok(); ++i) {
                float x = lidar_datas[num].points[i].x;
                float y = lidar_datas[num].points[i].y;
                float z = lidar_datas[num].points[i].z;
                
                // ignore the invalid point
                if(x == 0 && y == 0 && z == 0) {  
                    continue;
                }
                
                // set coordinate for the cloud point
                cloud->points[i].x = x;
                cloud->points[i].y = y;
                cloud->points[i].z = z;

                // set the RGB for the cloud point  
                int RGB[3] = {0, 0, 0}; 
                getColor(matrix_in, matrix_out, x, y, z, row, col, color_vector, RGB); 
                // ignore the unexisting point
                if (RGB[0] == 0 && RGB[1] == 0 && RGB[2] == 0) {  
                    continue;
                }
                
                cloud->points[i].r = RGB[0];
                cloud->points[i].g = RGB[1];
                cloud->points[i].b = RGB[2];

            }
            // once lidar_datas receive something new, it will transform it into a ROS cloud type
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(*cloud, output);

            output.header.frame_id = "livox_frame"; 
            pub.publish(output); // publish the cloud point to rviz
            loop_rate.sleep();
            ++num;

        }
        // clean the data when the process is finished
        else if(num >= 10) {
            lidar_datas.clear();
            num = 0;
            ROS_INFO("Finish all the process");
            break;
        }
    }
    
    return 0;
}




