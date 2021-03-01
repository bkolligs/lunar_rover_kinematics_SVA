#pragma once
#include "kinematics.h"
#include <gtest/gtest.h>


class OmegaTest: public ::testing::Test {
    
    protected:
        Eigen::Matrix<double, 10, 1> q0 = Eigen::Matrix<double, 10, 1>::Zero();
        Kinematics testRover{0.1, 0.1, 0.1, 0.1, q0};
        Eigen::Matrix3d m;
        Eigen::Vector3d ori;

};