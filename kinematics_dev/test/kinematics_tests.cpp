// test fixtures
#include "SkewTest.h"
#include "OmegaTest.h"
#include "HomogTest.h"
#include "TransformTest.h"
#include "JacobianTest.h"
#include "MotionTest.h"
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

TEST_F(MotionTest, MotionTestActuationOne){
    // initial position
     q0 << 0,
           0,
      3.1416,
     -1.9951,
      3.0029,
      0.1700,
    103.6662,
    112.6422,
    103.6662,
    112.6422;

    // desired body speed
    qdotInput << 0,
                 0,
                 0,
                 3,
                 0,
                 0,
                 0,
                 0,
                 0,
                 0;

    compare << 0,
               0,
          3.1416,
         -2.0251,
          3.0029,
          0.1700,
        104.0948,
        113.0708,
        104.0948,
        113.0708;

    Kinematics testRover = {0.1, 0.2, 0.1, 0.07, q0};

    direction = ACTUATION;
    deltaT = 0.01;
    testRover.motionPrediction(qdotInput, deltaT, direction);

    ASSERT_TRUE(testRover.getState().isApprox(compare, 0.0001));
}

TEST_F(MotionTest, MotionTestActuationTwo){
        // initial position
     q0 << 0,
           0,
      3.1416,
     -1.9951,
      3.0029,
      0.1700,
    103.6662,
    112.6422,
    103.6662,
    112.6422;

    // desired body speed
    qdotInput << 0,
                 0,
                 0,
             -1000,
                 0,
                 0,
                 0,
                 0,
                 0,
                 0,

    compare << 0,
               0,
          3.1416,
          8.0049,
          3.0029,
          0.1700, 
        -39.1909,
        -30.2149,
        -39.1909,
        -30.2149;

    Kinematics testRover = {0.1, 0.2, 0.1, 0.07, q0};

    direction = ACTUATION;
    deltaT = 0.01;
    testRover.motionPrediction(qdotInput, deltaT, direction);

    ASSERT_TRUE(testRover.getState().isApprox(compare, 0.0001));
}

TEST_F(MotionTest, MotionTestActuationYawAndX){
        // initial position
     q0 << 0,
           0,
      3.1416,
     -1.9951,
      3.0029,
      0.1700,
    103.6662,
    112.6422,
    103.6662,
    112.6422;

    // desired body speed
    qdotInput << 0,
                 0,
           -3.1416,
           -1.0000,
                 0,
                 0,
                 0,
                 0,
                 0,
                 0;

    compare << 0,
               0,
          3.1102,
         -1.9851,
          3.0029,
          0.1700,
        103.5683,
        112.4545,
        103.5683,
        112.4545;

    Kinematics testRover = {0.1, 0.2, 0.1, 0.07, q0};

    direction = ACTUATION;
    deltaT = 0.01;
    testRover.motionPrediction(qdotInput, deltaT, direction);

    ASSERT_TRUE(testRover.getState().isApprox(compare, 0.0001));
}

TEST_F(MotionTest, MotionTestNavigationOne){
        // initial position all zeros

    // desired body speed
    qdotInput << 0,
                 0,
                 0,
                 0,
                 0,
                 0,
                 3,
                 2,
                 4,
                 3;

    compare << -0.0000,
                0.0000,
               -0.0007,
                0.0021,
                0.0000,
                0.1700,
                0.0300,
                0.0200,
                0.0400,
                0.0300;

    Kinematics testRover = {0.1, 0.2, 0.1, 0.07, q0};

    direction = NAVIGATION;
    deltaT = 0.01;
    testRover.motionPrediction(qdotInput, deltaT, direction);

    ASSERT_TRUE(testRover.getState().isApprox(compare, 0.0001));
}

TEST_F(MotionTest, MotionTestNavigationTwo){
        // initial position all zeros

    // desired body speed
    qdotInput << 0,
                 0,
                 0,
                 0,
                 0,
                 0,
           -3.6000,
            4.6000,
           -4.2100,
            5.0000;

    compare << 0.0000,
              -0.0000,
               0.0061,
               0.0003,
              -0.0000,
               0.1700,
              -0.0360,
               0.0460,
              -0.0421,
               0.0500;

    Kinematics testRover = {0.1, 0.2, 0.1, 0.07, q0};

    direction = NAVIGATION;
    deltaT = 0.01;
    testRover.motionPrediction(qdotInput, deltaT, direction);

    ASSERT_TRUE(testRover.getState().isApprox(compare, 0.0001));
}

TEST_F(MotionTest, MotionSequenceActuationForward){
    direction = ACTUATION;
    deltaT = 0.01;

    int simStart = 0;
    int simEndSec = 10;
    int simEnd = simEndSec/deltaT;

    Kinematics testRover = {0.1, 0.2, 0.1, 0.07, q0};

    for (int t = 0; t <= simEnd; t ++){
        qdotInput << 0,
                     0,
                     0,
                     5,
                     0,
                     0,
                     0,
                     0,
                     0,
                     0;

        testRover.motionPrediction(qdotInput, deltaT, direction);
    }

    compare << 0,
               0,
               0,
         50.0500,
               0,
          0.1700,
        715.0000,
        715.0000,
        715.0000,
        715.0000;

    ASSERT_TRUE(testRover.getState().isApprox(compare, 0.0001));
}

TEST_F(MotionTest, MotionSequenceActuationRotate){
    direction = ACTUATION;
    deltaT = 0.01;

    int simStart = 0;
    int simEndSec = 10;
    int simEnd = simEndSec/deltaT;

    Kinematics testRover = {0.1, 0.2, 0.1, 0.07, q0};

    for (int t = 0; t <= simEnd; t ++){
        qdotInput << 0,
                     0,
                2*M_PI,
                     0,
                     0,
                     0,
                     0,
                     0,
                     0,
                     0;

        testRover.motionPrediction(qdotInput, deltaT, direction);
    }

    compare << 0,
               0,
         62.8947,
               0,
               0,
          0.1700,
        -89.8495,
         89.8495,
        -89.8495,
         89.8495;

    ASSERT_TRUE(testRover.getState().isApprox(compare, 0.0001));
}

TEST_F(MotionTest, MotionSequenceActuationTurn){
    direction = ACTUATION;
    deltaT = 0.01;

    int simStart = 0;
    int simEndSec = 10;
    int simEnd = simEndSec/deltaT;
    float driveArcRadius = 0.5;
    float driveArcVelocity = M_PI/4;
    float driveArcSecond = 4;
    float driveArcPsiDot = driveArcVelocity/(driveArcRadius*driveArcSecond);
    float driveArcXDot = driveArcVelocity/driveArcSecond;

    Kinematics testRover = {0.1, 0.2, 0.1, 0.07, q0};

    for (int t = 0; t <= simEnd; t ++){
        if (t > 0 && t <= 0.1*simEnd){
            qdotInput << 0,
                         0,
                         0,
                         1,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0;     
        }
        else if (t > 0.1*simEnd && t<=0.5*simEnd){
            qdotInput << 0,
                         0,
            driveArcPsiDot,
              driveArcXDot,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0;     
        }
        else if (t > 0.5*simEnd && t<=0.7*simEnd){
            qdotInput << 0,
                         0,
                         0,
                         1,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0;    
        }

        else{
            qdotInput = Eigen::Matrix<double, 10, 1>::Zero();
        }

        testRover.motionPrediction(qdotInput, deltaT, direction);
    }

    compare << 0,
               0,
          1.5708,
          1.5010,
          2.4990,
          0.1700,
         51.8331,
         56.3211,
         51.8331,
         56.3211;


    ASSERT_TRUE(testRover.getState().isApprox(compare, 0.0001));
}

TEST_F(MotionTest, MotionSequenceNavigation){
    direction = NAVIGATION;
    deltaT = 0.01;

    int simStart = 0;
    int simEndSec = 10;
    int simEnd = simEndSec/deltaT;

    Kinematics testRover = {0.1, 0.2, 0.1, 0.07, q0};

    for (int t = 0; t <= simEnd; t ++){
        if (t > 0 && t <= 0.25*simEnd){
            qdotInput << 0,
                         0,
                         0,
                         0,
                         0,
                         0,
                      M_PI,
                      M_PI,
                      M_PI,
                      M_PI;   
        }
        else if (t > 0.25*simEnd && t<=0.5*simEnd){
            qdotInput << 0,
                         0,
                         0,
                         0,
                         0,
                         0,
                    2*M_PI,
                   -2*M_PI,
                    2*M_PI,
                   -2*M_PI;    
        }
        else if (t > 0.5*simEnd && t<=0.75*simEnd){
            qdotInput << 0,
                         0,
                         0,
                         0,
                         0,
                         0,
                      M_PI,
                      M_PI,
                      M_PI,
                      M_PI;     
        }
        else if (t > 0.75*simEnd && t<=0.9*simEnd){
            qdotInput << 0,
                         0,
                         0,
                         0,
                         0,
                         0,
                         0,
                    4*M_PI,
                         0,
                    6*M_PI;     
        }
        else{
            qdotInput << 0,
                         0,
                         0,
                         0,
                         0,
                         0,
                      M_PI,
                      M_PI,
                      M_PI,
                      M_PI;
        }

        testRover.motionPrediction(qdotInput, deltaT, direction);
    }

    compare << 0.0000,
              -0.0000,
              -0.5498,
               0.5553,
              -1.2818,
               0.1700,
              34.5889,
              22.0226,
              34.5889,
              31.4473;


    ASSERT_TRUE(testRover.getState().isApprox(compare, 0.0001));
}
 
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}