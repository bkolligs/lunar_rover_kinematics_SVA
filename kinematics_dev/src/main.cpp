#include "kinematics.h"
#include "classTest.h"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <array>

class TestRoverClass {
    double w_ = 0.1;
    double l_ = 0.2;
    double h_ = 0.1;
    double r_ = 0.07;

    public:
        Kinematics * classRover;

        TestRoverClass(double w, double l, double h, double r, Eigen::Matrix<double, 10, 1>  init){
            double w_ = w;
            double l_ = l;
            double h_ = h;
            double r_ = r;

            Kinematics rover(w_, l_, h_, r_, init);
            classRover = &rover;

        }
};

void testEigenMatrixCompare(Kinematics & rover){
    Eigen::Vector3d orientation;
    Eigen::Vector3d position;
    Eigen::Matrix4d testInitializer = Eigen::Matrix4d::Zero();
    Eigen::Matrix4d homogTransform = Eigen::Matrix4d::Zero();

    orientation << M_PI/6, 0, 0;
    position << 0.001, 0.1, 1;

    homogTransform = rover.homogenousTransform(orientation, position);

    testInitializer  <<    1.0,     0,     0,      0.001, 
                             0, 0.866,  -0.5,      0.1,
                             0,   0.5, 0.866,      1,
                             0,     0,     0,      1;

    std::cout << "Testing the norm function. "<< std::endl << homogTransform.norm() << std::endl << testInitializer.norm() << std::endl << "Diff: " << (homogTransform - testInitializer).norm()  << " Same: " << homogTransform.isApprox(testInitializer, 0.001) << std::endl;

    std::cout << "Kinematics Testing!" << std::endl;
    std::cout << homogTransform  << std::endl;
    std::cout << testInitializer  << std::endl;
}

void testEigenSlice(Kinematics & rover){
    Eigen::Matrix4d sliceTest = Eigen::Matrix4d::Identity(4, 4);
    Eigen::Matrix<double, 10, 1> vectorSlice = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    Eigen::Matrix<double, 4, 3> configurationTable;
    Eigen::Vector3d ori;
    Eigen::Vector3d pos;
    Eigen::Vector3d tableBlock;

    double l_ = 0.1;
    double w_ = 0.2;
    double h_ = 0.3;

    // rows i is wheel i
    configurationTable <<  l_,  w_, -h_,
                           l_, -w_, -h_,
                          -l_,  w_, -h_,
                          -l_, -w_, -h_;

    ori = rover.getState()(Eigen::seq(0, 2));
    pos = rover.getState()(Eigen::seq(3, 5));
    tableBlock = configurationTable.block(1, 0, 1, 3).transpose();

    std::cout << "Matrix slice: " << std::endl << sliceTest(Eigen::seq(0, 2), Eigen::seq(0, 2)) << std::endl;
    std::cout << "Vector slice: " << std::endl << ori << std::endl << pos << std::endl;
    std::cout << "Configuration Table Slice: " << std::endl << tableBlock << std::endl;

}

void testEigenMatrixArray(){
    Eigen::Matrix4d matOne = Eigen::Matrix4d::Zero();
    Eigen::Matrix4d matTwo = Eigen::Matrix4d::Zero();
    Eigen::Matrix4d matThree = Eigen::Matrix4d::Zero();
    std::array<Eigen::Matrix4d*, 3> listOfMatrices = {&matOne, &matTwo, &matThree};

    matOne(0, 0) = 1.0;
    matTwo(1, 1) = 2.0;
    matThree(2, 2) = 3.0;

    for (std::array<Eigen::Matrix4d*, 3>::iterator it = listOfMatrices.begin(); it != listOfMatrices.end();){

        std::cout << (**it) << std::endl;
        it++;
    }
}

void printTransformList(Kinematics &rover){
    std::cout << "Rover Transform List" << std::endl;
    std::array<Eigen::Matrix4d, 9> roverList = rover.getTransforms();

        for (std::array<Eigen::Matrix4d, 9>::iterator it = roverList.begin(); it != roverList.end();){

        std::cout << (*it) << std::endl;
        it++;
    }

}

// template so that I can show the frobenius norms for various sized eigen matrices
template <typename T>
void frobeniusNorm(T m, T compare){
    double mNorm;
    double compareNorm;
    double diffNorm;
    double precision;

    mNorm = m.norm();
    compareNorm = compare.norm();
    diffNorm = (m - compare).norm();
    precision = 1;

    std::cout << "Comparing: " << std::endl << m << std::endl << compare << std::endl;
    std::cout << m - compare << std::endl;
    std::cout << "M Norm: " << mNorm << " Compare Norm: "<< compareNorm << std::endl;
    std::cout << "Diff Norm: " << diffNorm << " Same Matrix? " << (diffNorm <= std::min(mNorm, compareNorm)) << std::endl;

}

void currentSizes(){
    std::cout << "Size of Eigen 4d array: " << sizeof(Eigen::Matrix4d) << std::endl;
    std::cout << "Size of Kinematics class: " << sizeof(Kinematics) << std::endl;
}

void testRoverTransformsAndJacobian(){
    Eigen::Matrix<double, 10, 1> q1 = Eigen::Matrix<double, 10, 1>::Zero();

    Eigen::Matrix4d compare;

    q1 <<      0,
               0,
          3.1416,
        - 1.9951,
          3.0029,
          0.1700,
        103.6662,
        112.6422,
        103.6662,
        112.6422;

    Kinematics rover(0.1, 0.2, 0.1, 0.07, q1);
    compare << -1.0000,   -0.0000,        0,   -1.9951,
                0.0000,   -1.0000,        0,    3.0029,
                    0,         0,    1.0000,    0.1700,
                    0,         0,         0,    1.0000;

    frobeniusNorm(rover.getTransforms()[0], compare);
    std::cout << rover.getTransforms()[0].isApprox(compare, 0.01) << std::endl;
    
    printTransformList(rover);

    std::cout << "Rover Jacobian: " << std::endl << rover.getJacobian() << std::endl;
    rover.jacobian();

    std::cout << "Rover Jacobian: " << std::endl << rover.getJacobian() << std::endl;

}


void testEigenMatrixPointers(){
    // goal is to evaluate whether its more space efficient to assign blocks to a matrix3d, or use a pointer to a block object
    // can't take a pointer to a block because it is temporary in the function call

    Eigen::Matrix4d compare = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d * p_compare = &compare;
    Eigen::Matrix3d blockCopy = compare.block(0, 0, 3, 3);
    Eigen::Block<Eigen::Matrix4d> block = compare.block(0, 0, 3, 3);
    Eigen::Block<Eigen::Matrix4d> * p_block = &block;



    // test out constant pointers
    const Eigen::Matrix4d * cp_compare = &compare;
    Eigen::Matrix3d c_block = cp_compare->block(0, 0, 3, 3);
    // can't call a pointer directly to the block object because it is taking an address of a temporary variable :(
    const Eigen::Block<const Eigen::Matrix4d> cp_block = cp_compare->block(0, 0, 3, 3);
    // vector size compare
    Eigen::Vector3d testVec = compare.block(0, 3, 3, 1);
    // so basically the block is constant size...so I think assignment  to vector and matrix is fine
    const Eigen::Block<const Eigen::Matrix4d> cp_blockVector = cp_compare->block(0, 3, 3, 1);


    // change block to see if these are aliased or copied
    block(2, 2) = 5.0;
    // this also works
    (*p_block)(0, 0) = 9.0;

    std::cout << "Pointer to Matrix: " << p_compare << std::endl;
    std::cout << "Pointer to block: " << p_block << std::endl;
    std::cout << "Deref Pointer to block slice: " << std::endl << *p_block << std::endl;
    std::cout << "Block objects are aliased: " << std::endl << block << std::endl << compare << std::endl;
    std::cout << "Matrix4d Block size: " << sizeof(block) << ". Size of pointer to Matrix4d block: " << sizeof(p_block) << ". Size of Matrix3d: " << sizeof(blockCopy) << std::endl;
    std::cout << "Size of constant vector block: " << sizeof(cp_blockVector) << ". Size of Vector3d: " << sizeof(testVec) << std::endl; 

}

void testJacobianPinv(){
    Eigen::Matrix<double, 10, 1> q1 = Eigen::Matrix<double, 10, 1>::Zero();

    Eigen::Matrix4d compare;

    q1 <<      0,
               0,
          3.1416,
        - 1.9951,
          3.0029,
          0.1700,
        103.6662,
        112.6422,
        103.6662,
        112.6422;

    Kinematics rover(0.1, 0.2, 0.1, 0.07, q1);

    rover.jacobian();

    std::cout << "Rover Jacobian: " << std::endl << rover.getJacobian() << std::endl;
    std::cout << "Eigen Pinv: " << std::endl << rover.getJacobian().completeOrthogonalDecomposition().pseudoInverse() << std::endl;
    std::cout << "Eigen Pinv Shape: " << rover.getJacobian().completeOrthogonalDecomposition().pseudoInverse().rows() << rover.getJacobian().completeOrthogonalDecomposition().pseudoInverse().cols() << std::endl; 
}

void testMotionPrediction(){
    Eigen::Matrix<double, 10, 1> q1 = Eigen::Matrix<double, 10, 1>::Zero();
    Eigen::Matrix<double, 10, 1> q1dot = Eigen::Matrix<double, 10, 1>::Zero();

    Eigen::Matrix<double, 10, 1> compare;

    q1 <<      0,
               0,
          3.1416,
        - 1.9951,
          3.0029,
          0.1700,
        103.6662,
        112.6422,
        103.6662,
        112.6422;

    q1dot << 0,
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

    Kinematics rover(0.1, 0.2, 0.1, 0.07, q1);

    // test actuation
    KinematicDirection direction = ACTUATION;

    std::cout << "Rover State: " << std::endl << rover.getState() << std::endl;
    rover.motionPrediction(q1dot, 0.01, direction);
    std::cout << "Rover State: " << std::endl << rover.getState() << std::endl;
    std::cout << "Correct? " << compare.isApprox(rover.getState(), 0.0001) << std::endl;
    // check the norms
    frobeniusNorm<Eigen::Matrix<double, 10, 1>>(rover.getState(), compare);
}

void testMotionPredictionWithClass(){
    Eigen::Matrix<double, 10, 1> q1 = Eigen::Matrix<double, 10, 1>::Zero();
    Eigen::Matrix<double, 10, 1> q1dot = Eigen::Matrix<double, 10, 1>::Zero();

    Eigen::Matrix<double, 10, 1> compare;

    q1 <<      0,
               0,
          3.1416,
        - 1.9951,
          3.0029,
          0.1700,
        103.6662,
        112.6422,
        103.6662,
        112.6422;

    q1dot << 0,
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

    TestRoverClass rover {0.1, 0.2, 0.3, 0.07, q1};

    // test actuation
    KinematicDirection direction = ACTUATION;

    std::cout << "Rover State: " << std::endl << rover.classRover->getState() << std::endl;
    rover.classRover->motionPrediction(q1dot, 0.01, direction);
    std::cout << "Rover State: " << std::endl << rover.classRover->getState() << std::endl;
    std::cout << "Correct? " << compare.isApprox(rover.classRover->getState(), 0.0001) << std::endl;
    // check the norms
    frobeniusNorm<Eigen::Matrix<double, 10, 1>>(rover.classRover->getState(), compare);
}

int main(int argc, char** argv)
{
    Eigen::Matrix<double, 10, 1> q0 = Eigen::Matrix<double, 10, 1>::Zero();
    Eigen::Matrix<double, 10, 1> q1 = Eigen::Matrix<double, 10, 1>::Zero();

    Eigen::Matrix3d omega;

    q0.block(6, 0, 4, 1) << 0.1, 0.4, 0.1, 0.2;

    std::array<double, 10> testArrayAssign = {0, 0, 0, 0, 0, 1, 2, 3, 4, 5};
    Eigen::Matrix<double, 10, 1> q2 {testArrayAssign.data()};

    // std::cout << q2 << std::endl;


    Kinematics rover(0.1, 0.2, 0.1, 0.07, q0);

    testMotionPredictionWithClass();
    // testMotionPrediction();
    // testJacobianPinv();
    // testRoverTransformsAndJacobian();
    // testEigenMatrixPointers();
    // testEigenMatrixArray();
    // testEigenSlice(rover);
    // currentSizes();


    return 0;
}