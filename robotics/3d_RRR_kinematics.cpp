// Markus Buchholz 2022

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <vector>

using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::Vector3f;
using Eigen::VectorXf;



MatrixXf identity()
{

    MatrixXf I(3, 3);

    I(0, 0) = 1;
    I(0, 1) = 0;
    I(0, 2) = 0;

    I(1, 0) = 0;
    I(1, 1) = 1;
    I(1, 2) = 0;

    I(2, 0) = 0;
    I(2, 1) = 0;
    I(2, 2) = 1;

    return I;
}

MatrixXf DH(double alfa, double theta, double r, double d)
{

    MatrixXf dh(4, 4);

    dh(0, 0) = std::cos(theta);
    dh(0, 1) = -std::sin(theta) * std::cos(alfa);
    dh(0, 2) = std::sin(theta) * std::sin(alfa);
    dh(0, 3) = r * std::cos(theta);

    dh(1, 0) = std::sin(theta);
    dh(1, 1) = std::cos(theta) * std::cos(alfa);
    dh(1, 2) = -std::cos(theta) * std::sin(alfa);
    dh(1, 3) = r * std::sin(theta);

    dh(2, 0) = 0;
    dh(2, 1) = std::sin(alfa);
    dh(2, 2) = std::cos(alfa);
    dh(2, 3) = d;

    dh(3, 0) = 0;
    dh(3, 1) = 0;
    dh(3, 2) = 0;
    dh(3, 3) = 1;

    return dh;
}

//-------------------------------------------------------
// ----------Build Jacobia- RRR robot all joints revolute
//-------------------------------------------------------

/*
    | J1(3,1) | J2(3,1) | J3(3,1) |
    | J4(3,1) | J6(3,1) | J6(3,1) |
    */
// DH(double alfa, double theta, double r, double d)

MatrixXf jacobian(double theta1, double theta2, double theta3)
{

    double alfa1 = M_PI / 2;
    double alfa2 = 0;
    double alfa3 = 0;

    double r1 = 0;
    double r2 = 1;
    double r3 = 1;

    double d1 = 1;
    double d2 = 0;
    double d3 = 0;

    MatrixXf DH00(4, 4);
    MatrixXf DH01(4, 4);
    MatrixXf DH12(4, 4);
    MatrixXf DH23(4, 4);

    // DH00 = 0;
    DH01 = DH(alfa1, theta1, r1, d1);
    DH12 = DH(alfa2, theta2, r2, d2);
    DH23 = DH(alfa3, theta3, r3, d3);

    MatrixXf D01 = DH01;
    MatrixXf D01R = D01.block<3, 3>(0, 0);
    MatrixXf D01T = D01.block<3, 1>(0, 3);

    MatrixXf D02 = DH01 * DH12;
    MatrixXf D02R = D02.block<3, 3>(0, 0);
    MatrixXf D02T = D02.block<3, 1>(0, 3);

    MatrixXf D03 = DH01 * DH12 * DH23;
    MatrixXf D03R = D03.block<3, 3>(0, 0);
    MatrixXf D03T = D03.block<3, 1>(0, 3);

    Vector3f Ri(0, 0, 1);

    Vector3f vecD01R = D01R * Ri;
    Vector3f vecD02R = D02R * Ri;

    Vector3f vecD01T(Map<Vector3f>(D01T.data(), D01T.cols() * D01T.rows()));
    Vector3f vecD02T(Map<Vector3f>(D02T.data(), D01T.cols() * D02T.rows()));
    Vector3f vecD03T(Map<Vector3f>(D03T.data(), D03T.cols() * D03T.rows()));

    MatrixXf J1 = Ri.cross(vecD03T); //R00
    MatrixXf J2 = (vecD01R).cross(vecD03T - vecD01T);
    MatrixXf J3 = (vecD02R).cross(vecD03T - vecD02T);

    MatrixXf J(3, 3); // we consider only linear velocities

    J(0, 0) = J1(0, 0);
    J(1, 0) = J1(1, 0);
    J(2, 0) = J1(2, 0);

    J(0, 1) = J2(0, 0);
    J(1, 1) = J2(1, 0);
    J(2, 1) = J2(2, 0);

    J(0, 2) = J3(0, 0);
    J(1, 2) = J3(1, 0);
    J(2, 2) = J3(2, 0);

    return J;
}

MatrixXf computePseudoInverse(float theta1, float theta2, float theta3)
{
    MatrixXf J = jacobian(theta1, theta2, theta3);
    MatrixXf invJ = J.completeOrthogonalDecomposition().pseudoInverse();

    return invJ;
}

MatrixXf FWD_Kinematics(float theta1, float theta2, float theta3)
{

    double alfa1 = M_PI / 2;
    double alfa2 = 0;
    double alfa3 = 0;

    double r1 = 0;
    double r2 = 1;
    double r3 = 1;

    double d1 = 1;
    double d2 = 0;
    double d3 = 0;

    MatrixXf DH00(4, 4);
    MatrixXf DH01(4, 4);
    MatrixXf DH12(4, 4);
    MatrixXf DH23(4, 4);

    DH01 = DH(alfa1, theta1, r1, d1);
    DH12 = DH(alfa2, theta2, r2, d2);
    DH23 = DH(alfa3, theta3, r3, d3);

    MatrixXf DH03 = DH01 * DH12 * DH23;

    MatrixXf FWD = DH03.block<3, 1>(0, 3);

    return FWD;
}

MatrixXf applyAlgorithm()
{

    MatrixXf e(3, 1);
    MatrixXf Xd(3, 1);
  
    MatrixXf i_1_theta(3, 1);
    MatrixXf i_theta(3, 1);

    i_theta(0, 0) = 0;
    i_theta(1, 0) = 0;
    i_theta(2, 0) = 0;
    
    MatrixXf FWD = FWD_Kinematics(i_theta(0, 0), i_theta(1, 0), i_theta(2, 0));

   
    Xd(0, 0) = 0.6;
    Xd(1, 0) = 0.9;
    Xd(2, 0) = 2;
    
    e = Xd - FWD;

    while ((std::abs(e(0, 0)) > 0.00001) || (std::abs(e(1, 0)) > 0.00001) || (std::abs(e(2, 0)) > 0.00001))
    {

        MatrixXf invJ = computePseudoInverse(i_theta(0, 0), i_theta(1, 0), i_theta(2, 0));

        i_1_theta = i_theta + invJ * e;

        FWD = FWD_Kinematics(i_1_theta(0, 0), i_1_theta(1, 0), i_1_theta(2, 0));

        e = Xd - FWD;
        i_theta = i_1_theta;

       // std::cout << i_theta * (180 / M_PI) << std::endl;
    }

    return i_theta;
}

int main()
{

    MatrixXf thetaRobot = applyAlgorithm();
    MatrixXf FWD = FWD_Kinematics(thetaRobot(0, 0), thetaRobot(1, 0), thetaRobot(2, 0));
    std::cout << FWD << "\n";
}
