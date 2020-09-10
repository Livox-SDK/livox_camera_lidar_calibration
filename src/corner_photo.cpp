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
#include <sys/types.h>
#include <dirent.h>

#include "common.h"

using namespace std;
using namespace cv;

void writeData(const string imagename, const string filename, const float x, const float y, uint mode);

cv::Mat gray_img, src_img;
cv::RNG  random_number_generator;
string photo_path, output_name, intrinsic_path;

void getParameters() {
    cout << "Get the parameters from the launch file" << endl;

    if (!ros::param::get("input_photo_folder_path", photo_path)) {
        cout << "Can not get the value of input_photo_folder_path" << endl;
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

void writeData(const string imagename, const string filename, const float x, const float y, uint mode) {
    ofstream outfile(filename.c_str(), ios_base::app);
    if (!outfile) {
        cout << "Can not open the file: " << filename << endl;
        exit(0);
    }
    switch(mode) {
        case(0):
            outfile << imagename << endl;
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

vector<cv::Point2f> corners;

void InputCallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if (event == EVENT_LBUTTONDOWN){
        cout << "x:" << x << " y:" << y << endl;
        cv::Point2f p;
        p.x = x;
        p.y = y;
        corners.push_back(p);
    }
}


bool naturalSortCompare(string i, string j) {
    string str_i = i.substr(0,i.find("."));
    string str_j = j.substr(0,j.find("."));
    //cout << i << " " << str_i<< " " << str_i.length()<< endl;
    if (str_i.length() == 0 || str_j.length()== 0){
        return false;
    }
    try
    {
        return stoi(str_i) < stoi(str_j);
    }
    catch (invalid_argument const &e)
    {
        return false;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cornerPhoto");
    getParameters();

    vector<string> files;
    DIR* dirp = opendir(photo_path.c_str());
    struct dirent * dp;
    while ((dp = readdir(dirp)) != NULL) {
        files.push_back(dp->d_name);
    }
    // Sort files list by natural counting order
    for(unsigned i=0; i < files.size(); i++){
        sort(files.begin(), files.end(), naturalSortCompare);
    }
    closedir(dirp);

    cout << "\n\nImage path is set to:\n" << photo_path << endl;
    cout << "...Ready for corner picking..." << endl;
    for(unsigned i=0; i < files.size(); i++){
        if(files.at(i).compare("..") == 0 || files.at(i).compare(".") == 0){
            continue;
        }

        string image_path = photo_path + "/" + files.at(i); 
        src_img = cv::imread(image_path);

        if(src_img.empty()) {
            cout << image_path <<"is NOT a valid image ! Skipping" << endl;
            continue;
        }

        cout << "\n\nOpening undistorted Image of: " << files.at(i) << endl;

        // Read intrinsic and distortion
        vector<float> intrinsic;
        getIntrinsic(intrinsic_path, intrinsic);
        vector<float> distortion;
        getDistortion(intrinsic_path, distortion);

        // Set intrinsic parameters of the camera
        cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        cameraMatrix.at<double>(0, 0) = intrinsic[0];
        cameraMatrix.at<double>(0, 2) = intrinsic[2];
        cameraMatrix.at<double>(1, 1) = intrinsic[4];
        cameraMatrix.at<double>(1, 2) = intrinsic[5];

        // Set radial distortion and tangential distortion
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

        cout << "Starting from top-left in clockwise direction, click 4 corners of the board on the image" << endl;
        cout << "5th click to any location finalizes corner picking" << endl;
        cout << "While Image has focus:" << endl;
        cout << "Press \"s\" to skip this image (No point must be selected to skip)" << endl;
        cout << "Press \"r\" to remove the last point" << endl;
        // cv::namedWindow("source", CV_WINDOW_KEEPRATIO);
        cv::namedWindow("Source: " + files.at(i));
        //set the callback function for any mouse event
        setMouseCallback("Source: " + files.at(i), InputCallBackFunc, NULL);
        cv::imshow("Source: " + files.at(i), src_img);

        // Loop while not all 4 points are clicked
        bool skip_image = false;
        while (corners.size() == 0 || corners.size() % 5 != 0){
            int keypress = cv::waitKey(100);
            if (keypress == 115 ){      // "s"  is pressed 
                if (corners.size() == 0){
                    skip_image = true;
                    break;
                }else{
                    cout << "Cannot skip image while there are selected points, remove them first" << endl;
                }
            }
            if (keypress == 114){       // "r"  is pressed
                if (corners.size() > 0){
                    corners.pop_back();
                    cout << "Removed last point" << endl;
                }else{
                    cout << "No point is left to remove !!" << endl;
                }
            }

            // Displaying points
            if (corners.size() == 0 ){
                cv::imshow("Source: " + files.at(i), src_img);
            }else{
                cv::Size winSize = cv::Size(5, 5);
                cv::Size zerozone = cv::Size(-1, -1);
                cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 40, 0.001);
                
                cv::namedWindow("Source: " + files.at(i));
                cv::cvtColor(src_img, gray_img, cv::COLOR_BGR2GRAY);
                cv::cornerSubPix(gray_img, corners, winSize, zerozone, criteria);

                // Display resulting image for confirmation
                cv::Mat result_img = src_img.clone();
                for(uint t = 0; t < corners.size(); ++t) {
                    cv::circle(result_img, corners[t], 3, cv::Scalar(random_number_generator.uniform(0, 255), random_number_generator.uniform(0, 255), random_number_generator.uniform(0, 255)), 1, 8, 0);
                }
                cv::namedWindow("Source: " + files.at(i));
                cv::imshow("Source: " + files.at(i), result_img);
            }
        }
        // End corner picking
        cv::destroyWindow("Source: " + files.at(i));
        // Skip current image
        if (skip_image){
            continue;
        };

        // Remove last click as its just for confirmation
        corners.pop_back();

        // Generating results
        cout << "While Image has focus: \nPress \"r\" for repeat same image \nPress \"n\" to confirm and move to next image" << endl;
        cv::Size winSize = cv::Size(5, 5);
        cv::Size zerozone = cv::Size(-1, -1);
        cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 40, 0.001);
        
        cv::namedWindow("Output: " + files.at(i));
        cv::cvtColor(src_img, gray_img, cv::COLOR_BGR2GRAY);
        cv::cornerSubPix(gray_img, corners, winSize, zerozone, criteria);

        // Display resulting image for confirmation
        cv::Mat result_img = src_img.clone();
        for(uint t = 0; t < corners.size(); ++t) {
            cv::circle(result_img, corners[t], 3, cv::Scalar(random_number_generator.uniform(0, 255), random_number_generator.uniform(0, 255), random_number_generator.uniform(0, 255)), 1, 8, 0);
        }
        cv::namedWindow("Output: " + files.at(i));
        cv::imshow("Output: " + files.at(i), result_img);
        bool repeatImage = false;
        while(true){
            int keypress = cv::waitKey(0);
            if (keypress == 114){       // "r"  is pressed
                repeatImage = true;
                break;
            }
            else if (keypress == 110){  // "n"  is pressed
                break;
            }else{
                cout << "Press \"r\" for repeat the image or \"n\" for next image" << endl;
            }
        }
        cv::destroyWindow("Output: " + files.at(i));
        if (repeatImage){ 
            cout << "Repeating image..." << endl;
            i--;
            corners.clear();
            continue;
        }

        // Confirming complete writing results to file        
        for(uint t = 0; t < corners.size(); ++t) {
            printf("(%.3f %.3f)", corners[t].x, corners[t].y);
            writeData(files.at(i), output_name, corners[t].x, corners[t].y, t);
        }

        cout << endl << "Result saved for \"" << files.at(i) << "\"!" << endl;
        corners.clear();
    }
    return 0;
}







