#pragma once
#include "kinematics.h"
#include <gtest/gtest.h>


class MotionTest: public ::testing::Test {
    
    protected:
        Eigen::Matrix<double, 10, 1> q0 = Eigen::Matrix<double, 10, 1>::Zero();
        Eigen::Matrix<double, 10, 1> qdotInput = Eigen::Matrix<double, 10, 1>::Zero();

        Eigen::Matrix<double, 10, 1> m;
        Eigen::Matrix<double, 10, 1> compare;

        double deltaT = 0.01;
        KinematicDirection direction = ACTUATION;
       

    public:

        MotionTest(){
             q0(5) = 0.17;
        }
};



