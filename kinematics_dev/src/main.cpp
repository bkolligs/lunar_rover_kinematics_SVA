#include "kinematics.h"
#include "classTest.h"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <array>


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

void frobeniusNorm(Eigen::Matrix4d m, Eigen::Matrix4d compare){
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
    std::cout << "Diff Norm: " << diffNorm << " Same Matrix: " << (diffNorm <= std::min(mNorm, compareNorm)) << std::endl;

}

int main(int argc, char** argv)
{
    Eigen::Matrix<double, 10, 1> q0 = Eigen::Matrix<double, 10, 1>::Zero();
    Eigen::Matrix<double, 10, 1> q1 = Eigen::Matrix<double, 10, 1>::Zero();
    Eigen::Matrix4d compare;
    Eigen::Matrix3d omega;

    q0.block(6, 0, 4, 1) << 0.1, 0.4, 0.1, 0.2;

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


    printTransformList(rover);
    frobeniusNorm(rover.getTransforms()[0], compare);
    std::cout << rover.getTransforms()[0].isApprox(compare, 0.01) << std::endl;
    // std::cout << rover.homogenousTransform(q1(Eigen::seq(0, 2)), q1(Eigen::seq(3, 5))) << std::endl;
    // testEigenMatrixArray();
    // testEigenSlice(rover);
    // std::cout << "Size of Eigen 4d array: " << sizeof(Eigen::Matrix4d) << std::endl;
    // std::cout << "Size of Kinematics class: " << sizeof(Kinematics) << std::endl;

    return 0;
}