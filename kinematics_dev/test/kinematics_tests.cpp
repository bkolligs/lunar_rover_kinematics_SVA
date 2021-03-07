// test fixtures
#include "SkewTest.h"
#include "OmegaTest.h"
#include "HomogTest.h"
#include "TransformTest.h"
#include "JacobianTest.h"
// general headers
#include <gtest/gtest.h>
#include <iostream>

// *******************************************************************************************************
// Kinematics::skew() tests
TEST_F(SkewTest, PositiveValues){
    // check with non zero values
    v << 1, 2, 3;
    m = testRover.skew(v);
    ASSERT_TRUE(m.transpose().isApprox(-m));
}

TEST_F(SkewTest, ZeroValues){
    // check zero values
    v << 0, 0, 0;
    m = testRover.skew(v);
    ASSERT_TRUE(m.transpose().isApprox(-m));
}

TEST_F(SkewTest, NegativeValues){
    v << -1, -2, -3;
    m = testRover.skew(v);
    ASSERT_TRUE(m.transpose().isApprox(-m));
}

// *******************************************************************************************************
// Kinematics::calculateOmega() tests
TEST_F(OmegaTest, ZeroValue){
    // check with zero values
    ori << 0, 0, 0;
    testRover.calculateOmega(ori, m);
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
    testRover.calculateOmega(ori, m);
    Eigen::Matrix3d compare = Eigen::Matrix3d::Zero();

    // calculated with MATLAB, comparing to 3 sigfigs
    compare << 1.0, 0.4010,  0.9484,
                 0, 0.9211, -0.3894,
                 0, 0.5589,  1.3220;

    // print out the matrices
    // std::cout << compare << std::endl;
    // std::cout << m << std::endl;


    ASSERT_TRUE(m.isApprox(compare, 0.01));
}

TEST_F(OmegaTest, NonZeroValue2){
    // check with non zero values
    ori << 0.4, M_PI, -M_PI;
    testRover.calculateOmega(ori, m);
    Eigen::Matrix3d compare = Eigen::Matrix3d::Zero();

    // calculated with MATLAB, comparing to 3 sigfigs
    compare << 1.0,       0,       0, 
                 0,  0.9211, -0.3894,
                 0, -0.3894, -0.9211;

    // print out the matrices
    // std::cout << compare << std::endl;
    // std::cout << m << std::endl;


    ASSERT_TRUE(m.isApprox(compare, 0.01));
}

TEST_F(OmegaTest, NonZeroValuePI){
    // check with non zero values
    ori << M_PI, M_PI, -M_PI;
    testRover.calculateOmega(ori, m);
    Eigen::Matrix3d compare = Eigen::Matrix3d::Zero();

    // calculated with MATLAB, comparing to 3 sigfigs
    compare << 1.0,    0,   0,
                 0, -1.0,   0, 
                 0,    0, 1.0;
    // print out the matrices
    // std::cout << compare << std::endl;
    // std::cout << m << std::endl;


    ASSERT_TRUE(m.isApprox(compare, 0.01));
}

// *******************************************************************************************************
// Kinematics::homogeneousTransform tests
TEST_F(HomogTest, RotationCorrectX){
    ori << M_PI, 0, 0;
    pos << 0, 0, 0;

    compare <<   1.0,   0,   0,   0, 
                   0,-1.0,   0,   0,
                   0,   0,-1.0,   0,
                   0,   0,   0, 1.0;

    m = testRover.homogenousTransform(ori, pos);

    ASSERT_TRUE(m.isApprox(compare, 0.01));

    ori <<  M_PI/6, 0, 0;
    pos << 0.001, 0.1, 1;
    compare <<    1.0,     0,     0, 0.001, 
                    0, 0.866,  -0.5,   0.1,
                    0,   0.5, 0.866,     1,
                    0,     0,     0,     1;
 
    m = testRover.homogenousTransform(ori, pos);

    ASSERT_TRUE(m.isApprox(compare, 0.001));
}

TEST_F(HomogTest, RotationCorrectY){
    ori << 0, M_PI, 0;
    pos << 0, 0, 0;

    compare <<     0,   0, 1.0,   0, 
                   0, 1.0,   0,   0,
                -1.0,   0,   0,   0,
                   0,   0,   0, 1.0;

    m = testRover.homogenousTransform(ori, pos);

    ASSERT_TRUE(m.isApprox(compare, 1));

    ori << 0, M_PI/6, 0;
    pos << 0.001, 0.1, 0;
    compare <<    0.866, 0, 0.5, 0.001, 
                    0, 1,     0,   0.1,
                 -0.5, 0, 0.866,     0,
                    0, 0,     0,     1;

    m = testRover.homogenousTransform(ori, pos);

    ASSERT_TRUE(m.isApprox(compare, 0.01));
}

TEST_F(HomogTest, RotationCorrectZ){
    ori << 0, 0, M_PI;
    pos << 0, 0, 0;

    compare << -1.0,    0,   0,   0, 
                  0, -1.0,   0,   0,
                  0,    0, 1.0,   0,
                  0,    0,   0, 1.0;
 
    m = testRover.homogenousTransform(ori, pos);

    ASSERT_TRUE(m.isApprox(compare, 0.01));

    ori << 0, 0, M_PI / 6;
    pos << 0.001, 0.1, 1;
    compare << 0.866,    -0.5, 0, 0.001, 
                 0.5, 0.866, 0,   0.1,
                   0,     0, 1,     1,
                   0,     0, 0,     1;

    m = testRover.homogenousTransform(ori, pos);

    ASSERT_TRUE(m.isApprox(compare, 0.001));
}

TEST_F(HomogTest, FullEulerAngles){
    ori << M_PI/3, -M_PI/6, M_PI/6;
    pos << 2, 3, 5;
    compare << 0.7500,  -0.6250,   0.2165,    2.0000,
               0.4330,   0.2165,  -0.8750,    3.0000,
               0.5000,   0.7500,   0.4330,    5.0000,
                    0,        0,        0,    1.0000;
    
    m = testRover.homogenousTransform(ori, pos);

    ASSERT_TRUE(m.isApprox(compare, 0.001));

}

// *******************************************************************************************************
// Kinematics::updateTransforms tests
TEST_F(TransformTest, TransformListTestWorld){
    // test world to body transform
    m = testRover.getTransforms()[0];
    compare << -1.0000,   -0.0000,        0,   -1.9951,
                0.0000,   -1.0000,        0,    3.0029,
                    0,         0,    1.0000,    0.1700,
                    0,         0,         0,    1.0000;

    ASSERT_TRUE(m.isApprox(compare, 0.0001));
}

TEST_F(TransformTest, TransformListTestJoint){

    // test a joint location transform
    m = testRover.getTransforms()[1];
    compare <<   1.0000,   -0.0000,   -0.0063,   -2.1951,
                -0.0000,   -1.0000,    0.0000,    2.9029,
                -0.0063,         0,   -1.0000,    0.0700,
                      0,         0,         0,    1.0000;

    ASSERT_TRUE(m.isApprox(compare, 0.0001));
}

TEST_F(TransformTest, TransformListTestContact){

    // test a contact frame transform
    m = testRover.getTransforms()[5];
    compare << -1.0000,   -0.0000,        0,   -2.1951,
                0.0000,   -1.0000,        0,    2.9029,
                     0,         0,   1.0000,         0,
                     0,         0,        0,    1.0000;

    ASSERT_TRUE(m.isApprox(compare, 0.0001));
}



TEST_F(JacobianTest, JacobianMatrixTest){
	testRover.jacobian();
	m = testRover.getJacobian();
    compare <<
         0,   -0.1700,   -0.1000,    1.0000,         0,         0,   -0.0700,         0,         0,         0,
    0.1700,         0,    0.2000,         0,    1.0000,         0,         0,         0,         0,         0,
    0.1000,   -0.2000,         0,         0,         0,    1.0000,         0,         0,         0,         0,
         0,   -0.1700,    0.1000,    1.0000,         0,         0,         0,   -0.0700,         0,         0,
    0.1700,         0,    0.2000,         0,    1.0000,         0,         0,         0,         0,         0,
   -0.1000,   -0.2000,         0,         0,         0,    1.0000,         0,         0,         0,         0,
         0,   -0.1700,   -0.1000,    1.0000,         0,         0,         0,         0,   -0.0700,         0,
    0.1700,         0,   -0.2000,         0,    1.0000,         0,         0,         0,         0,         0,
    0.1000,    0.2000,         0,         0,         0,    1.0000,         0,         0,         0,         0,
         0,   -0.1700,    0.1000,    1.0000,         0,         0,         0,         0,         0,   -0.0700,
    0.1700,         0,   -0.2000,         0,    1.0000,         0,         0,         0,         0,         0,
   -0.1000,    0.2000,         0,         0,         0,    1.0000,         0,         0,         0,         0;

   ASSERT_TRUE(m.isApprox(compare, 0.0001));
}
 
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}