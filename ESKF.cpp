//
// Created by msi on 2020/7/16.
//

#include "ESKF.h"

void ESKF::init_state_nominal(MotionData input) {
    this->curr_state=MotionData2State(input);
    this->curr_state.g<<0,0,-9.81;
    normalizeQ(curr_state.q);
    this->timestamp=input.timestamp;
}

void ESKF::log_state_nominal() {
    log_State_nominal(this->curr_state);
}

void ESKF::predict(Imu_msg imu_msg_) {
    if(imu_msg_.timestamp== this->timestamp)
        return;

    _predict_err_covar(imu_msg_);
    _predict_nominal_state(imu_msg_);
    this->timestamp=imu_msg_.timestamp;
}

void ESKF::_predict_err_covar(Imu_msg imu_msg_) {

    V3d w_m=imu_msg_.gyro_m;
    V3d a_m=imu_msg_.acc_m;
    V3d a_b=curr_state.acc_bias;
    V3d w_b=curr_state.gyro_bias;
    Eigen::Quaterniond q = curr_state.q;
    M3d R = q.matrix();

    // 状态转移矩阵
    Eigen::MatrixXd F=Eigen::MatrixXd::Identity(err_state.Vec.size(),err_state.Vec.size());
    // 系统矩阵
    Eigen::MatrixXd A=Eigen::MatrixXd::Zero(err_state.Vec.size(),err_state.Vec.size());

//    A.block<3,3>(0,3)=M3d::Identity();
//    A.block<3,3>(3,6)=-R*skew_matrix(a_m-a_b);
//    A.block<3,3>(3,9)=-R;
//    A.block<3,3>(3,15)=M3d::Identity();
//    A.block<3,3>(6,6)=-skew_matrix(w_m-w_b);
//    A.block<3,3>(6,12)= -M3d::Identity();

    A.block<3,3>(0,6)=M3d::Identity();
    A.block<3,3>(6,3)=-R*skew_matrix(a_m-a_b);
    A.block<3,3>(6,9)=-R;
//    A.block<3,3>(3,15)=M3d::Identity();
    A.block<3,3>(3,3)=-skew_matrix(w_m-w_b);
    A.block<3,3>(3,12)= -M3d::Identity();

    // 耗时主要在这里
//    TicToc tic;
    double dt=imu_msg_.timestamp-this->timestamp;
    Eigen::MatrixXd Fdt =  A*dt;
    Eigen::MatrixXd Fdt2= Fdt * Fdt;
    Eigen::MatrixXd Fdt3= Fdt2 * Fdt;

    // 使用3阶截断近似得到状态转移矩阵 (292)
    F= F + Fdt + 0.5*Fdt2 +(1.0/6.0)*Fdt3;

    // 更新协方差
    Eigen::MatrixXd Qc_dt= 0.5*dt*this->Q_init;
    this->P=F*(this->P+Qc_dt)*F.transpose()+Qc_dt;
}

void ESKF::_predict_nominal_state(Imu_msg imu_msg_) {
    V3d p = curr_state.p;
    Eigen::Quaterniond q = curr_state.q;
    V3d v = curr_state.v;
    V3d a_b = curr_state.acc_bias;
    V3d w_b = curr_state.gyro_bias;
    V3d g = curr_state.g;

    V3d w_m = imu_msg_.gyro_m;
    V3d a_m = imu_msg_.acc_m;
    double dt = imu_msg_.timestamp- this->timestamp;

    // 转换成轴角 （实际上没必要）
    // 实际上: angle*w_ = w_m-w_b
    //double angle = (w_m-w_b).norm();
    //V3d w_ = (w_m-w_b)/angle;
    //M3d dq_half_R = Sophus::SO3d::exp(0.5*dt*angle*w_).matrix();
    //Eigen::Quaterniond dq_half= Eigen::Quaterniond(dq_half_R);
    //M3d dq_R = Sophus::SO3d::exp(dt*angle*w_).matrix();
    //Eigen::Quaterniond dq= Eigen::Quaterniond(dq_R);

    //使用近似:  (把下面的"w_m-w_b"替换成"angle*w_"也是一样的)
    Eigen::Quaterniond dq_half = theta2q(w_m-w_b,0.5*dt);
    Eigen::Quaterniond dq= theta2q(w_m-w_b,dt);
    Eigen::Quaterniond q_half_next=q*dq_half;
    Eigen::Quaterniond q_next=q*dq;
    normalizeQ(q_half_next);
    normalizeQ(q_next);

    // RK4更新位置和速度
    M3d R = q.matrix();
    M3d R_half_next= (q*dq_half).matrix();
    M3d R_next = curr_state.q.matrix();

    V3d v_k1 = R * (a_m-a_b) + g;
    V3d v_k2 = R_half_next * (a_m-a_b) + g;
    V3d v_k3 = v_k2;
    V3d v_k4 = R_next * (a_m-a_b)+g;
    V3d v_next = v+ dt * (v_k1 + 2 * v_k2 + 2 * v_k3 + v_k4) / 6;

    // 位置
    V3d p_k1 = v;
    V3d p_k2 = v+0.5*dt*v_k1;
    V3d p_k3 = v+0.5*dt*v_k2;
    V3d p_k4 = v+dt*v_k3;
    V3d p_next = p + dt * (p_k1 + 2 * p_k2 + 2 * p_k3 + p_k4) / 6;

    curr_state.p=p_next;
    curr_state.v=v_next;
    curr_state.q=q_next;
    curr_state.timestamp=imu_msg_.timestamp;
}

void ESKF::update(const MotionData &measure, const Measurement_noise &noise) {

    // 构造观测矩阵H
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6,err_state.Vec.size());      // 6x18

    H.block<3,3>(0,0)=M3d::Identity();          // 观测的位置 对 posi_t 求导
    H.block<3,3>(3,3)=M3d::Identity();          // 观测的姿态 对 xita_t 求导

    Eigen::MatrixXd P_HT_= this->P * H.transpose();             // 18x6


//
    // 噪声构成矩阵
    Eigen::MatrixXd V = Eigen::MatrixXd::Identity(6,6);
    V.block<3,3>(0,0)*=pow(noise.p_noise_sigma,2);
    V.block<3,3>(3,3)*=pow(noise.q_noise_sigma,2);
//
    // 计算卡尔曼增益
    //std::cout <<(H * P_HT_ + V).inverse()<<std::endl;
    Eigen::MatrixXd K=P_HT_ * ((H * P_HT_ + V).inverse());                  // 18x6

//    static int i=0;
//    if (i==0){
//        std::cout<<K<<std::endl;
//        i++;
//    }

    // 更新误差状态协方差矩阵P
    this->P = (Eigen::MatrixXd::Identity(18,18) - K*H)* this->P;
    // 对称化
    this->P = 0.5*(this->P+ this->P.transpose());

    Eigen::VectorXd delta=Eigen::VectorXd::Zero(6);
    delta.segment(0,3)=measure.twb-curr_state.p;    // 位置误差
//
    Eigen::Quaterniond q_m =  measure.q;
    normalizeQ(q_m);
    Eigen::Quaterniond q = curr_state.q;                    // norminal_state
    Eigen::Quaterniond q_delta = q.conjugate()*q_m;         // q_m 到 q的变换
//    Eigen::AngleAxisd angleAxis_delta(q_delta);
    normalizeQ(q_delta);
    double sin_theta =sqrt(q_delta.x()*q_delta.x()+
                           q_delta.y()*q_delta.y()+
                           q_delta.z()*q_delta.z()) ;
    double angle = asin(sin_theta);
    V3d axis;
    if(angle<1e-9){
        axis.setZero();
    }else{
        axis = V3d(q_delta.x(),q_delta.y(),q_delta.z())/sin_theta;
    }
    // 角度差
    delta.segment(3,3)=angle*axis;
//    std::cout<<angle*axis<<std::endl;

    // 根据卡尔曼增益，计算误差
    Eigen::VectorXd errors = K*delta;
//
    //误差状态注入到 nominal_state
    curr_state.p += errors.segment(0,3);
    //更新速度
    curr_state.v += errors.segment(6,3);

    // 四元数姿态更新
    // 方法一:
    //M3d RR = Sophus::SO3d::exp(errors.segment(3,3)).matrix();
    //curr_state.q = curr_state.q * Eigen::Quaterniond(RR);
    // 方法二: 使用近似
    curr_state.q = curr_state.q * Eigen::Quaterniond(
            theta2q(errors.segment(3,3)));

    // 更新acc_bias,gyro_bias
    curr_state.acc_bias += errors.segment(9,3);
    curr_state.gyro_bias+= errors.segment(12,3);

    // 更新重力
    curr_state.g += errors.segment(15,3);

    curr_state.timestamp=measure.timestamp;

    //重置误差状态为零，更新误差状态协方差P
    this->G.setIdentity(18,18);
    G.block<3,3>(3,3)=M3d::Identity()-skew_matrix(0.5*errors.segment(3,3));
    this->P = G * this->P * G.transpose();
}

const MotionData ESKF::getCurrState() {
    MotionData data=State2MotionData(curr_state);
    return data;
}
