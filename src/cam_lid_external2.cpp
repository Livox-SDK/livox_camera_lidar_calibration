#include <ros/ros.h>
#include <iostream>
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <fstream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <string>

#include "common.h"
#include "result_verify.h"

using namespace std;

typedef pcl::PointXYZRGB PointType;
string lidar_path, photo_path, intrinsic_path, extrinsic_path;
int error_threshold;
vector<float> init;

void getParameters();

class external_cali {
public:
    external_cali(PnPData p) {
        pd = p;
    }

    template <typename T>
    bool operator()(const T *_q, const T *_t, const T *_in, T *residuals) const {
        Eigen::Matrix<T, 3, 3> in_incre;
        in_incre << _in[0], T(0), _in[1], T(0), _in[2], _in[3], T(0), T(0), T(1);
        
        Eigen::Quaternion<T> q_incre{_q[ 3 ], _q[ 0 ], _q[ 1 ], _q[ 2 ]};
        Eigen::Matrix<T, 3, 1> t_incre{_t[ 0 ], _t[ 1 ], _t[ 2 ]};

        Eigen::Matrix<T, 3, 1> p_l(T(pd.x), T(pd.y), T(pd.z));
        Eigen::Matrix<T, 3, 1> p_c = q_incre.toRotationMatrix() * p_l + t_incre;
        Eigen::Matrix<T, 3, 1> p_2 = in_incre*p_c;

        residuals[0] = p_2[0]/p_2[2] - T(pd.u);
        residuals[1] = p_2[1]/p_2[2] - T(pd.v);

        return true;
    }

    static ceres::CostFunction *Create(PnPData p) {
        return (new ceres::AutoDiffCostFunction<external_cali, 2, 4, 3, 4>(new external_cali(p)));
    }


private:
    PnPData pd;

};

void getParameters() {
    cout << "Get the parameters from the launch file" << endl;

    if (!ros::param::get("input_lidar_path", lidar_path)) {
        cout << "Can not get the value of input_lidar_path" << endl;
        exit(1);
    }
    if (!ros::param::get("input_photo_path", photo_path)) {
        cout << "Can not get the value of input_photo_path" << endl;
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
    if (!ros::param::get("error_threshold", error_threshold)) {
        cout << "Can not get the value of error_threshold" << endl;
        exit(1);
    }
    init.resize(12);
    if (!ros::param::get("init_value", init)) {
        cout << "Can not get the value of init_value" << endl;
        exit(1);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "getExt2");
    getParameters();

    vector<PnPData> pData;
    getData(lidar_path, photo_path, pData);
    
    Eigen::Matrix3d R;  
    R << init[0], init[1], init[2],
         init[4], init[5], init[6],
         init[8], init[9], init[10];
    Eigen::Quaterniond q(R); 
    double ext[7];
    double in[4];

    ext[0] = q.x();
    ext[1] = q.y();
    ext[2] = q.z();
    ext[3] = q.w();
    ext[4] = init[3];  
    ext[5] = init[7];  
    ext[6] = init[11];

    vector<float> intrin;
    getIntrinsic(intrinsic_path, intrin);
    in[0] = intrin[0];
    in[1] = intrin[2];
    in[2] = intrin[4];
    in[3] = intrin[5];

    Eigen::Map<Eigen::Quaterniond> m_q = Eigen::Map<Eigen::Quaterniond>(ext);
    Eigen::Map<Eigen::Vector3d> m_t = Eigen::Map<Eigen::Vector3d>(ext + 4);

    ceres::LocalParameterization * q_parameterization = new ceres::EigenQuaternionParameterization();
    ceres::Problem problem;

    problem.AddParameterBlock(ext, 4, q_parameterization);
    problem.AddParameterBlock(ext + 4, 3);
    problem.AddParameterBlock(in, 4);
  
    for(auto val : pData) {
        ceres::CostFunction *cost_function;
        cost_function = external_cali::Create(val);
        problem.AddResidualBlock(cost_function, NULL, ext, ext + 4, in);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.BriefReport() << endl << endl;

    Eigen::Matrix3d rot = m_q.toRotationMatrix();
    writeExt(extrinsic_path, rot, m_t);

    cout << "Extrinsic parameters is:" << endl;
    cout << rot << endl;
    cout << m_t << endl << endl; 
    cout << "New intrinsic parameters is:" << endl;
    int in_count = 0;
    for(int i = 0; i < 9; ++i) {
        if (i == 0 || i == 2 || i == 4 || i == 5) {
            string valueStr = double2str(in[in_count]);
            int length = valueStr.size();
            cout << in[in_count];
            ++in_count;
            for (int i = 15; i > length; --i) {
                cout << " ";
            }
        }
        else if (i == 8) {
            cout << 1;
            cout << endl;
        }
        else {
            cout << 0;
            for (int i = 0; i < 14; ++i) {
                cout << " ";
            }
        }
        if((i + 1)%3 == 0) {
            cout << endl;
        }
    }

    cout << "Use the extrinsic result to reproject the data" << endl;
    float error[2] = {0, 0};
    vector<float> intrinsic(9, 0);
    intrinsic[0] = in[0];
    intrinsic[2] = in[1];
    intrinsic[4] = in[2];
    intrinsic[5] = in[3];
    intrinsic[8] = 1;
    getUVErrorNewIntrinsic(extrinsic_path, lidar_path, photo_path, error, error_threshold, intrinsic);
    
    cout << "u average error is: " << error[0] << endl;
    cout << "v average error is: " << error[1] << endl;
    if (error[0] + error[1] < error_threshold) {
        cout << endl << "The reprojection error smaller than the threshold, extrinsic result seems ok" << endl;
    }
    else {
        cout << endl << "The reprojection error bigger than the threshold, extrinsic result seems not ok" << endl;
    }
    return 0;  
}


