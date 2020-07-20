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
        // 取状态维度
        int vec_size=err_state.Vec.size();
        // 矩阵初始化
        Fi=Eigen::MatrixXd::Zero(vec_size, vec_size);
        Eigen::MatrixXd tmp_Q_init=Eigen::MatrixXd::Zero(vec_size,vec_size);
        P=Eigen::MatrixXd::Zero(vec_size,vec_size);

        // 式158,167 ， 加速度噪声积分得到的对 速度状态的影响
        tmp_Q_init.block<3,3>(0,0)=pow(param.acc_noise_sigma,2)*Eigen::Matrix3d::Identity();
        // 式159,167
        tmp_Q_init.block<3,3>(3,3)=pow(param.gyro_noise_sigma,2)*Eigen::Matrix3d::Identity();
        // 式160,167
        tmp_Q_init.block<3,3>(6,6)=pow(param.acc_bias_sigma,2)*Eigen::Matrix3d::Identity();
        // 式161,167
        tmp_Q_init.block<3,3>(9,9)=pow(param.gyro_bias_sigma,2)*Eigen::Matrix3d::Identity();
        // 式167
        Fi.block<3,3>(3, 3)=Eigen::Matrix3d::Identity();
        Fi.block<3,3>(6, 0)=Eigen::Matrix3d::Identity();
        Fi.block<3,3>(9, 6)=Eigen::Matrix3d::Identity();
        Fi.block<3,3>(12, 9)=Eigen::Matrix3d::Identity();
        Qi=(Fi * tmp_Q_init * Fi.transpose());
        P= 0.01 * Qi;
        log_Matrix("P:", Qi);
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
    Eigen::MatrixXd Qi;          // 噪声协方差矩阵(用于初始化的)
    Eigen::MatrixXd Fi;
    Eigen::MatrixXd P;

    std::deque<MotionData> predict_pose_;
    std::deque<MotionData> update_pose_;
};


#endif //ESKF_CPP_ESKF_H
