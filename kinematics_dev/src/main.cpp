#include "kinematics.h"
#include "classTest.h"
#include <iostream>
#include <eigen3/Eigen/Dense>


int main(int argc, char** argv)
{
    Eigen::Matrix<double, 10, 1> q0 = Eigen::Matrix<double, 10, 1>::Zero();
    Eigen::VectorXd q1(10);
    q1 = Eigen::VectorXd::Zero(10);
    Eigen::Vector3d orientation;
    Eigen::Vector3d position;
    Eigen::Matrix4d testInitializer = Eigen::Matrix4d::Zero();
    Eigen::Matrix4d homogTransform = Eigen::Matrix4d::Zero();

    orientation << M_PI/6, 0, 0;
    position << 0.001, 0.1, 1;

    Kinematics rover(0.1, 0.2, 0.3, 0.4, q0);
    homogTransform = rover.homogenousTransform(orientation, position);

    testInitializer  <<    1.0,     0,     0,      0.001, 
                             0, 0.866,  -0.5,      0.1,
                             0,   0.5, 0.866,      1,
                             0,     0,     0,      1;

    std::cout << "Testing the norm function. "<< std::endl << homogTransform.norm() << std::endl << testInitializer.norm() << std::endl << "Diff: " << (homogTransform - testInitializer).norm()  << " Same: " << homogTransform.isApprox(testInitializer, 0.001) << std::endl;

    std::cout << "Kinematics Testing!" << std::endl;
    std::cout << homogTransform  << std::endl;
    std::cout << testInitializer  << std::endl;
    return 0;
}