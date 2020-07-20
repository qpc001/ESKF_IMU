//
// Created by msi on 2020/7/16.
//

#include "utils.h"

void log_State_nominal(State_nominal input) {
    std::cout<<"State_nominal Log:"<<std::endl;
    std::cout<<"position: "<<input.p.transpose()<<std::endl;
    std::cout<<"volicity: "<<input.v.transpose()<<std::endl;
    std::cout<<"Quaterniond: \n"<<input.q.toRotationMatrix()<<std::endl;
    std::cout<<"Acc_bias: "<<input.acc_bias.transpose()<<std::endl;
    std::cout<<"Gyro_bias: "<<input.gyro_bias.transpose()<<std::endl;
    std::cout<<"g: "<<input.g.transpose()<<std::endl;
}

Eigen::Quaterniond theta2q(const Eigen::Vector3d theta) {
    Eigen::Vector3d dtheta_half = theta / 2.0;
    Eigen::Quaterniond dq;
    dq.w() = 1;
    dq.x() = dtheta_half.x();
    dq.y() = dtheta_half.y();
    dq.z() = dtheta_half.z();
    dq.normalize();
    return dq;
}
Eigen::Quaterniond theta2q(Eigen::Vector3d gyro, double delta_t) {
    // 方法一
//    Eigen::Matrix3d a=Sophus::SO3d::hat(gyro*delta_t);
//    return Eigen::Quaterniond(Sophus::SO3d::exp(gyro*delta_t).matrix());

    //方法二
    Eigen::Vector3d dtheta_half = gyro * delta_t / 2.0;
    Eigen::Quaterniond dq;
    dq.w() = 1;
    dq.x() = dtheta_half.x();
    dq.y() = dtheta_half.y();
    dq.z() = dtheta_half.z();
    dq.normalize();
    return dq;
}

Eigen::Matrix3d q2RotMat(Eigen::Quaterniond q) {
    return q.matrix();
}

void LoadData(std::string filename, std::deque<Imu_msg> &imu_data) {
    std::ifstream f;
    f.open(filename.c_str());

    if (!f.is_open()) {
        std::cerr << " can't open LoadFeatures file " << std::endl;
        return;
    }

    while (!f.eof()) {

        std::string s;
        std::getline(f, s);

        if (!s.empty()) {
            std::stringstream ss;
            ss << s;

            Imu_msg msg;
            double time;
            Eigen::Quaterniond q;
            Eigen::Vector3d t;
            Eigen::Vector3d gyro;
            Eigen::Vector3d acc;

            ss >> time;
            ss >> gyro(0);
            ss >> gyro(1);
            ss >> gyro(2);
            ss >> acc(0);
            ss >> acc(1);
            ss >> acc(2);


            msg.timestamp = time;         //时间戳
            msg.gyro_m = gyro;            //角速度
            msg.acc_m = acc;              //加速度

            imu_data.emplace_back(msg);

            /*pose.push_back(data);*/
        }
    }
}

void LoadData(std::string filename, std::deque<MotionData> &pose) {
    std::ifstream f;
    f.open(filename.c_str());

    if (!f.is_open()) {
        std::cerr << " can't open LoadFeatures file " << std::endl;
        return;
    }

    while (!f.eof()) {

        std::string s;
        std::getline(f, s);

        if (!s.empty()) {
            std::stringstream ss;
            ss << s;

            MotionData data;
            double time;
            Eigen::Quaterniond q;
            Eigen::Vector3d t;
            Eigen::Vector3d v;
            Eigen::Vector3d gyro;
            Eigen::Vector3d acc;

            ss >> time;
            ss >> t(0);
            ss >> t(1);
            ss >> t(2);
            ss >> q.w();
            ss >> q.x();
            ss >> q.y();
            ss >> q.z();
            ss >> v(0);
            ss >> v(1);
            ss >> v(2);

            data.timestamp = time;         //时间戳
            data.twb = t;
//            if(q.w()<0){
//                q.w()*=-1;
//                q.x()*=-1;
//                q.y()*=-1;
//                q.z()*=-1;
//            }
            data.Rwb = q.normalized().matrix();              //加速度
            data.q = q.normalized();
            data.imu_velocity= v;

            pose.emplace_back(data);

        }
    }
}

void SaveData(std::string outputFile, std::deque<MotionData> pose) {
    std::fstream f;
    f.open(outputFile.c_str(),std::ios::out|std::ios::trunc);

    if (!f.is_open()) {
        std::cerr << " Can't open Output file " << std::endl;
        return;
    }
    for(auto data : pose){
        f<< std::setiosflags(std::ios::fixed) << data.timestamp<<" ";
        f<< data.twb.x()<<" "<<
            data.twb.y()<<" "<<
            data.twb.z()<<" ";
        Eigen::Quaterniond q(data.Rwb);
        f<< q.x() <<" "<<
            q.x() <<" "<<
            q.y() <<" "<<
            q.z();
        f<< std::endl;
    }
    f.close();
    return;
}

State_nominal MotionData2State(MotionData input) {
    State_nominal state;
    state.p=input.twb;
    state.v=input.imu_velocity;
    state.q=input.q;
    state.acc_bias=input.imu_acc_bias;
    state.gyro_bias=input.imu_gyro_bias;
    state.timestamp=input.timestamp;
//    state.g<<(0,0,-9.81);
    return state;
}

void log_Matrix(std::string str,Eigen::MatrixXd mat) {
    std::cout<<str<<std::endl<<mat<<std::endl;
}

Eigen::Matrix3d skew_matrix(Eigen::Vector3d v) {
    Eigen::Matrix3d skew;
    skew<<  0 , -v.z(), v.y(),
            v.z(), 0 , -v.x(),
            -v.y(), v.x() , 0;
    //std::cout<<skew<<std::endl;
    return skew;
}

MotionData State2MotionData(State_nominal input) {
//    State_nominal state;
//    state.p=input.twb;
//    state.v=input.imu_velocity;
//    state.q=Eigen::Quaterniond(input.Rwb);
//    state.acc_bias=input.imu_acc_bias;
//    state.gyro_bias=input.imu_gyro_bias;
//    state.g<<(0,0,-9.81);

    MotionData data;
    data.twb=input.p;
    data.imu_velocity=input.v;
    data.Rwb=input.q.matrix();
    data.imu_acc_bias=input.acc_bias;
    data.imu_gyro_bias=input.gyro_bias;
    data.timestamp=input.timestamp;
    return data;
}

void normalizeQ(Eigen::Quaterniond &q) {
    if(q.w()<0){
        q.w()=-q.w();
        q.x()=-q.x();
        q.y()=-q.y();
        q.z()=-q.z();
    }
    q.normalize();
}

