#include "kinematics.h"
#include "classTest.h"
#include <iostream>
#include <eigen3/Eigen/Dense>


int main(int argc, char** argv)
{
    Eigen::Matrix<double, 10, 1> q0 = Eigen::Matrix<double, 10, 1>::Zero();
    Eigen::VectorXd q1(10);
    q1 = Eigen::VectorXd::Zero(10);

    Kinematics rover(0.1, 0.2, 0.3, 0.4, q0);


    std::cout << "Kinematics Testing!" << std::endl;
    std::cout << q1.transpose() << std::endl;
    return 0;
}