#include "kinematics.h"

// class constructor
Kinematics::Kinematics(double w, double l, double h, double r, Eigen::Matrix<double, 10, 1> q_initial) 
    : w_{w}, l_{l}, h_{h}, r_{r}, q_{q_initial}
    {
        // rows i is wheel i
        configurationTable <<  l_,  w_, -h_,
                               l_, -w_, -h_,
                              -l_,  w_, -h_,
                              -l_, -w_, -h_;

         // initialize transform list
        updateTransforms();

    }


// update the transforms
//TODO: Fix this list function
void Kinematics::updateTransforms(){
    Eigen::Matrix4d transWorld_Base;
    Eigen::Matrix4d transWorld_Joint;
    Eigen::Matrix4d transBase_Joint;
    Eigen::Matrix4d transWorld_Contact;
    Eigen::Matrix4d transBase_JointPrime;
    Eigen::Matrix4d transJoint_Contact;
    Eigen::Vector4d curJointValues;


    curWorldOri = q_(Eigen::seq(0, 2));
    curWorldPos = q_(Eigen::seq(3, 5));
    curJointValues = q_(Eigen::seq(6, 9));

    transWorld_Base = homogenousTransform(curWorldOri, curWorldPos);
    transformList[0] = transWorld_Base;

    // update the joint angle transforms and contact frame transforms
    for (int i = 1; i <= nWheels_;i++){
        // joint transform
        transBase_Joint = homogenousTransform({0, curJointValues(i-1), 0}, configurationTable.block(i-1, 0, 1, 3).transpose());
        transWorld_Joint = transWorld_Base * transBase_Joint;
        transformList[i] = transWorld_Joint;

        // contact transform for each wheel
        transBase_JointPrime = transBase_Joint;
        transBase_JointPrime.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity(3, 3);
        transJoint_Contact = homogenousTransform({0, 0, 0}, {0, 0, -r_});
        transWorld_Contact = transWorld_Base * transBase_JointPrime * transJoint_Contact;

        transformList[i + nWheels_] = transWorld_Contact;
    }
}

// calculate the jacobian
void Kinematics::jacobian(){

}

// navigation kinematics
void Kinematics::navigation(){

}

// actuation kinematics
void Kinematics::actuation(){

}

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
Eigen::Matrix<double, 10, 10> Kinematics::spatialToCartesian()
{
    Eigen::Matrix<double, 10, 10> vOutput = Eigen::Matrix<double, 10, 10>::Zero();
    Eigen::Matrix4d transWorld_Base = transformList[0];
    Eigen::Matrix3d rotWorld_Base = transWorld_Base.block(0, 0, 3, 3);
    Eigen::Matrix4d eyeFour = Eigen::Matrix4d::Identity(4, 4);
    Eigen::Matrix3d omega;

    calculateOmega(q_(Eigen::seq(0, 2)), omega);

    vOutput.block(0, 0, 3, 3) << omega;
    vOutput.block(3, 3, 3, 3) << rotWorld_Base;
    vOutput.block(6, 6, 4, 4) << eyeFour;

    return vOutput;

}

// calculate omega
void Kinematics::calculateOmega(const Eigen::Vector3d &orientation, Eigen::Matrix3d &omega){
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

Eigen::Matrix<double, 10, 1> & Kinematics::getState(){
    return q_;
}

std::array<Eigen::Matrix4d, 9> Kinematics::getTransforms(){
    return transformList;
}
