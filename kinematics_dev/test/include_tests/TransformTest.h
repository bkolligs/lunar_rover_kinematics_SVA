#pragma once
#include "kinematics.h"
#include <gtest/gtest.h>


class TransformTest: public ::testing::Test {
    
    protected:
        Eigen::Matrix<double, 10, 1> q0 = Eigen::Matrix<double, 10, 1>::Zero();

        Kinematics testRover{0.1, 0.2, 0.1, 0.07, q0};
        
        Eigen::Matrix4d m;
        Eigen::Matrix4d compare;
        Eigen::Vector3d ori;
        Eigen::Vector3d pos;
        double mNorm;
        double compareNorm;
        double diffNorm;
        double precision;

    public:
        TransformTest(){
            // test state
            q0 <<      0,
                       0,
                  3.1416,
                - 1.9951,
                  3.0029,
                  0.1700,
                103.6662,
                112.6422,
                103.6662,
                112.6422;
        }

};