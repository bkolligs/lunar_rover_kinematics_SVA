// tests
#include "SkewTest.h"
#include "OmegaTest.h"
// general headers
#include <gtest/gtest.h>
#include <iostream>

// *******************************************************************************************************
// Kinematics::skew() tests
TEST_F(SkewTest, PositiveValues){
    // check with non zero values
    v << 1, 2, 3;
    testRover.skew(v, m);
    ASSERT_TRUE(m.transpose().isApprox(-m));
}

TEST_F(SkewTest, ZeroValues){
    // check zero values
    v << 0, 0, 0;
    testRover.skew(v, m);
    ASSERT_TRUE(m.transpose().isApprox(-m));
}

TEST_F(SkewTest, NegativeValues){
    v << -1, -2, -3;
    testRover.skew(v, m);
    ASSERT_TRUE(m.transpose().isApprox(-m));
}

// *******************************************************************************************************
// Kinematics::omega() tests
TEST_F(OmegaTest, ZeroValue){
    // check with zero values
    ori << 0, 0, 0;
    testRover.omega(ori, m);
    Eigen::Matrix3d compare = Eigen::Matrix3d::Zero();
    compare(0, 0) = 1.0;
    compare(1, 1) = 1.0;
    compare(2, 2) = 1.0;

    // print out the identity
    // std::cout << compare << std::endl;

    ASSERT_TRUE(m.transpose().isApprox(compare));
}

TEST_F(OmegaTest, NonZeroValue1){
    // check with non zero values
    ori << 0.4, 0.8, 0.9;
    testRover.omega(ori, m);
    Eigen::Matrix3d compare = Eigen::Matrix3d::Zero();

    // calculated with MATLAB, comparing to 3 sigfigs
    compare << 1.0, 0.4010,  0.9484,
                 0, 0.9211, -0.3894,
                 0, 0.5589,  1.3220;

    // print out the matrices
    // std::cout << compare << std::endl;
    // std::cout << m << std::endl;


    ASSERT_TRUE(m.isApprox(compare, 3));
}

TEST_F(OmegaTest, NonZeroValue2){
    // check with non zero values
    ori << 0.4, M_PI, -M_PI;
    testRover.omega(ori, m);
    Eigen::Matrix3d compare = Eigen::Matrix3d::Zero();

    // calculated with MATLAB, comparing to 3 sigfigs
    compare(0, 0) = 1.0;
    compare(1, 1) = 1.0;
    compare(2, 2) = 1.0;

    // print out the matrices
    // std::cout << compare << std::endl;
    // std::cout << m << std::endl;


    ASSERT_TRUE(m.isApprox(compare, 3));
}

TEST_F(OmegaTest, NonZeroValuePI){
    // check with non zero values
    ori << M_PI, M_PI, -M_PI;
    testRover.omega(ori, m);
    Eigen::Matrix3d compare = Eigen::Matrix3d::Zero();

    // calculated with MATLAB, comparing to 3 sigfigs
    compare << 1.0,    0.0,     0.0,
                 0, 0.9211, -0.3894,
                 0, 0.3894, -0.9211;

    // print out the matrices
    // std::cout << compare << std::endl;
    // std::cout << m << std::endl;


    ASSERT_TRUE(m.isApprox(compare, 3));
}
 
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}