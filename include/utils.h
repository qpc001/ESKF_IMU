//
// Created by msi on 2020/7/16.
//
#pragma once
#ifndef ESKF_CPP_UTILS_H
#define ESKF_CPP_UTILS_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>
#include <vector>
#include <memory>
#include <string>
#include <fstream>
#include <deque>
#include <iomanip>
#include "tic_toc.hpp"
class utils {

};

struct Imu_msg{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp;
    Eigen::Vector3d acc_m;
    Eigen::Vector3d gyro_m;

    Imu_msg(){
        timestamp=0;
        acc_m.setZero();
        gyro_m.setZero();
    }
};

struct Imu_param{
    double frequency;
    double acc_noise_sigma;
    double gyro_noise_sigma;
    double acc_bias_sigma;
    double gyro_bias_sigma;
    Imu_param(){
        frequency=200;
        acc_noise_sigma=0.019;  // m/sqrt(s^3) (continuous noise)
        gyro_noise_sigma=0.015; // rad/sqrt(s) (continuous noise)
        acc_bias_sigma=0.0001; // m/sqrt(s^5)     (continuous bias)
        gyro_bias_sigma=2.0e-5; // rad/sqrt(s^3)   (continuous bias)
    }

};

struct State_nominal{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Quaterniond q;
    Eigen::Vector3d acc_bias;
    Eigen::Vector3d gyro_bias;
    Eigen::Vector3d g;
    double timestamp=0;
    State_nominal(){
        p.setZero();
        v.setZero();
        q.setIdentity();
        acc_bias.setZero();
        gyro_bias.setZero();
        g.setZero();
    }
};

struct State_error{
    Eigen::Matrix<double,18,1> Vec;
    Eigen::Ref<Eigen::Vector3d> p_delta=Vec.block(0, 0, 3, 1);
    Eigen::Ref<Eigen::Vector3d> v_delta=Vec.block(3, 0, 3, 1);
    Eigen::Ref<Eigen::Vector3d> xita_delta=Vec.block(6, 0, 3, 1);
    Eigen::Ref<Eigen::Vector3d> acc_bias_delta=Vec.block(9, 0, 3, 1);
    Eigen::Ref<Eigen::Vector3d> gyro_bias_delta=Vec.block(12, 0, 3, 1);
    Eigen::Ref<Eigen::Vector3d> g_delta = Vec.block(15, 0, 3, 1);
    State_error(){
        Vec.setZero();
    }
};

struct MotionData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp;
    Eigen::Quaterniond q;
    Eigen::Matrix3d Rwb;
    Eigen::Vector3d twb;
    Eigen::Vector3d imu_acc;
    Eigen::Vector3d imu_gyro;

    Eigen::Vector3d imu_gyro_bias;
    Eigen::Vector3d imu_acc_bias;

    Eigen::Vector3d imu_velocity;

    MotionData(){
        timestamp=0;
        Rwb.setIdentity();
        twb.setZero();
        imu_acc.setZero();
        imu_gyro.setZero();
        imu_gyro_bias.setZero();
        imu_acc_bias.setZero();
        imu_velocity.setZero();
    }
};

struct Measurement_noise{
    double p_noise_sigma=0.02;      // in meters    观测噪声，标准差
    double q_noise_sigma=0.015;     // in rad
};

static Eigen::Vector3d g(0,0,-9.81);

void log_State_nominal(State_nominal input);

void log_Matrix(std::string str,Eigen::MatrixXd mat);

Eigen::Quaterniond theta2q(const Eigen::Vector3d theta);
Eigen::Quaterniond theta2q(const Eigen::Vector3d gyro, double delta_t);

Eigen::Matrix3d q2RotMat(const Eigen::Quaterniond& q);

void normalizeQ(Eigen::Quaterniond &q);

Eigen::Matrix3d skew_matrix(Eigen::Vector3d);

void LoadData(std::string filename, std::deque<Imu_msg>& imu_msg);
void LoadData(std::string filename, std::deque<MotionData>& pose);
void SaveData(std::string outputFile,const std::deque<MotionData> pose);

State_nominal MotionData2State(const MotionData input);
MotionData State2MotionData(const State_nominal input);
#endif //ESKF_CPP_UTILS_H
