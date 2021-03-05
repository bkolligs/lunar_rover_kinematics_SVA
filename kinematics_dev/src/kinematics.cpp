#include "kinematics.h"

// class constructor
Kinematics::Kinematics(double w, double l, double h, double r, Eigen::Matrix<double, 10, 1> &q_initial) 
    : w_{w}, l_{l}, h_{h}, r_{r}, q_{q_initial}
    {

    }


// update the transforms
void updateTransforms(){

}

// calculate the jacobian
void jacobian(){

}

// navigation kinematics
void navigation(){

}

// actuation kinematics
void actuation(){

}

// theta is joint angle, pos is translation from parent frame
Eigen::Matrix4d Kinematics::homogenousTransform(const Eigen::Vector3d &ori, const Eigen::Vector3d &pos){
    Eigen::Matrix4d homogTransform = Eigen::Matrix4d::Zero();
    Eigen::Matrix3d rotationX;
    Eigen::Matrix3d rotationY;
    Eigen::Matrix3d rotationZ;

    double alpha = ori(0);
    rotationX << 1,          0,           0,
                 0, cos(alpha), -sin(alpha), 
                 0, sin(alpha),  cos(alpha);
    
    double beta = ori(1);
    rotationY << cos(beta), 0, sin(beta),
                         0, 1,         0,
                -sin(beta), 0, cos(beta);

    double gamma = ori(2);
    rotationZ << cos(gamma), -sin(gamma), 0,
                 sin(gamma),  cos(gamma), 0, 
                          0,           0, 1;

    // populate rotation of HT
    homogTransform(3, 3) = 1.0;
    homogTransform.block(0,0,3,3) << rotationZ * rotationY * rotationX;
    homogTransform.block(0, 3, 3, 1) << pos;

    return homogTransform;
}


// calculate V(q)
Eigen::Matrix<double, 10, 10> Kinematics::spatialToCartesian(const Eigen::Matrix<double, 10, 1> &q)
{
    Eigen::Matrix<double, 10, 10> vOutput = Eigen::Matrix<double, 10, 10>::Zero();
    Eigen::Matrix4d eyeFour = Eigen::Matrix4d::Identity(4, 4);
    Eigen::Matrix3d omega;

    omega(q_(Eigen::seq(0,3)), omega);


    return vOutput;

}

// calculate omega
void Kinematics::omega(const Eigen::Vector3d &orientation, Eigen::Matrix3d &omega){
    /*
    This function takes in an orientation 3vector and calculates the omega matrix
    args:
        orientation - 3vector of orientation values: phi, theta, psi
        omega - reference to output omega matrix
    */
    double phi = orientation(0);
    double theta = orientation(1);
    double psi = orientation(2);
    omega << 1, sin(phi)*tan(theta), cos(phi)*tan(theta),
             0,            cos(phi),           -sin(phi),
             0, sin(phi)/cos(theta), cos(phi)/cos(theta);
}

// skew symmetric matrix froma vector
void Kinematics::skew(const Eigen::Vector3d &v, Eigen::Matrix3d & skewSym)
{
    /*
    This function takes in a 3vector and modifies a predefined skew symmetric matrix
    args:
        v - 3vector to convert to skew symmetric
        skewSym - reference to skew symmetric matrix
    */
    skewSym <<   0, -v(2),  v(1),
              v(2),     0, -v(0),
             -v(1),  v(0),     0;
}