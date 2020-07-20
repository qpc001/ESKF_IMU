#include <iostream>
#include <deque>
#include "utils.h"
#include "ESKF.h"

int main() {
    std::deque<MotionData> pose_gt;
    std::deque<Imu_msg> imu_msgs;

    LoadData("../data/imu_noise.txt",imu_msgs);
    LoadData("../data/traj_gt.txt",pose_gt);

    std::cout<<"IMU Msg Size:"<<imu_msgs.size()<<std::endl;
    std::cout<<"Ground True Pose Size:"<<pose_gt.size()<<std::endl;

//    std::cout<< skew_matrix(Eigen::Vector3d(1,2,3))<<std::endl;
    ////////////////////////////ESKF//////////////////////
    Imu_param imu_param;
    ESKF::Ptr eskf_ptr;
    eskf_ptr=eskf_ptr->create(imu_param);

    int update_ratio=10;

    /// 1. init
    eskf_ptr->init_state_nominal(pose_gt.front());
    pose_gt.pop_front();
    imu_msgs.pop_front();
    eskf_ptr->log_state_nominal();

    /// 2.
    Measurement_noise mea_noise;
    for(int i=0;i<imu_msgs.size();i++){
        eskf_ptr->predict(imu_msgs[i]);

        if(i%update_ratio==0){
            MotionData measure=pose_gt[i];
            Eigen::Vector3d pos_noise=Eigen::Vector3d::Random()*mea_noise.p_noise_sigma;
            measure.twb += pos_noise;

            eskf_ptr->update(measure,mea_noise);
        }
        eskf_ptr->predict_pose_.emplace_back(eskf_ptr->getCurrState());
    }

    std::cout<<"Now Saving Pose Data"<<std::endl;
    SaveData("output.txt",eskf_ptr->predict_pose_);
    SaveData("traj_gt_out.txt",pose_gt);
    system("evo_traj tum ./output.txt --ref ./traj_gt_out.txt -p");
    return 0;
}
