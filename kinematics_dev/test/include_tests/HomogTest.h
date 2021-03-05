#pragma once
#include "kinematics.h"
#include <gtest/gtest.h>


class HomogTest: public ::testing::Test {
    
    protected:
        Eigen::Matrix<double, 10, 1> q0 = Eigen::Matrix<double, 10, 1>::Zero();
        Kinematics testRover{0.1, 0.1, 0.1, 0.1, q0};
        Eigen::Matrix4d m;
        Eigen::Matrix4d compare;
        Eigen::Vector3d ori;
        Eigen::Vector3d pos;

};