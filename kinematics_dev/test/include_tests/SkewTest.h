#pragma once
#include <gtest/gtest.h>
#include "kinematics.h"

// testing the skew() function
class SkewTest: public ::testing::Test{

    protected:
        Eigen::Matrix<double, 10, 1> q0 = Eigen::Matrix<double, 10, 1>::Zero();
        Kinematics testRover{0.1, 0.1, 0.1, 0.1, q0};
        Eigen::Matrix3d m;
        Eigen::Vector3d v;
        

};