//
// Created by gc on 18-11-30.
//

#ifndef PROJECT_COMPLEMENTARY_POSE_ESTIMATION_H
#define PROJECT_COMPLEMENTARY_POSE_ESTIMATION_H


#include <memory>
#include <ros/ros.h>
namespace hdl_localization {

    class ComplementaryPoseEstimation{

    public:
        ComplementaryPoseEstimation();

    private:
        double weight;



    };
}


#endif //PROJECT_COMPLEMENTARY_POSE_ESTIMATION_H
