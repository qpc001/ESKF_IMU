//
// Created by msi on 2020/7/16.
//

#ifndef ESKF_CPP_ESKF_H
#define ESKF_CPP_ESKF_H

#include "utils.h"
#include <vector>
#include <memory>
#include <string>
#include <fstream>
#include <sophus/se3.hpp>

typedef Eigen::Vector3d V3d;
typedef Eigen::Matrix3d M3d;

class ESKF {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::unique_ptr<ESKF> Ptr;

    inline static Ptr create(Imu_param param) {
        return std::unique_ptr<ESKF>(new ESKF(param));
    }

protected:
    ESKF(Imu_param param){
        int vec_size=err_state.Vec.size();
        G=Eigen::MatrixXd::Zero(vec_size,12);
        Eigen::MatrixXd tmp_Q_init=Eigen::MatrixXd::Zero(12,12);
        P=Eigen::MatrixXd::Zero(18,18);

        tmp_Q_init.block<3,3>(0,0)=pow(param.acc_noise_sigma,2)*Eigen::Matrix3d::Identity();
        tmp_Q_init.block<3,3>(3,3)=pow(param.gyro_noise_sigma,2)*Eigen::Matrix3d::Identity();
        tmp_Q_init.block<3,3>(6,6)=pow(param.acc_bias_sigma,2)*Eigen::Matrix3d::Identity();
        tmp_Q_init.block<3,3>(9,9)=pow(param.gyro_bias_sigma,2)*Eigen::Matrix3d::Identity();
        G.setZero();
        G.block<3,3>(3,3)=-Eigen::Matrix3d::Identity();
        G.block<3,3>(6,0)=-Eigen::Matrix3d::Identity();
        G.block<3,3>(9,6)=Eigen::Matrix3d::Identity();
        G.block<3,3>(12,9)=Eigen::Matrix3d::Identity();
        Q_init=(G*tmp_Q_init*G.transpose());
        P=0.01*Q_init;
        log_Matrix("P:",P);
    }

    // 禁用复制构造函数
    ESKF(const ESKF &) = delete;

    ESKF &operator=(const ESKF &) = delete;
public:
    void init_state_nominal(MotionData input);
    void log_state_nominal();

    void predict(Imu_msg imu_msg_);
    void _predict_err_covar(Imu_msg imu_msg_);
    void _predict_nominal_state(Imu_msg imu_msg_);

    void update(const MotionData& measure, const Measurement_noise &noise);

    const MotionData getCurrState();
private:
    int id;
    State_nominal curr_state;
    State_error err_state;
    double timestamp=0;
public:
    Eigen::MatrixXd Q_init;          // 噪声协方差矩阵(用于初始化的)
    Eigen::MatrixXd G;
    Eigen::MatrixXd P;

    std::deque<MotionData> predict_pose_;
    std::deque<MotionData> update_pose_;
};


#endif //ESKF_CPP_ESKF_H
