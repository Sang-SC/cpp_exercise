/*
* 文件功能：利用正运动学，通过改变关节角度，计算末端轨迹，并绘制出来
*/
#include <iostream>
#include <vector>
#include <cmath>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

struct JointAngles {
    double theta1;
    double theta2;
};

struct Point {
    double x;
    double y;
};

Point forwardKinematics(double theta1, double theta2, double link1, double link2) {
    Point endEffector;

    endEffector.x = link1 * cos(theta1) + link2 * cos(theta1 + theta2);
    endEffector.y = link1 * sin(theta1) + link2 * sin(theta1 + theta2);

    return endEffector;
}

std::vector<Point> computeArmTrajectory(double link1, double link2, double deltaTheta)
{
    std::vector<Point> trajectory;

    double theta1 = 0.0;
    double theta2 = 0.0;

    // link1 转动，link2 也会跟着转动
    while (theta1 <= M_PI / 2.0)
    {
        Point endEffector = forwardKinematics(theta1, theta2, link1, link2);

        trajectory.push_back({endEffector.x, endEffector.y});

        theta1 += deltaTheta;
    }

    // 只让 link2 转动
    while (theta2 <= M_PI)
    {
        Point endEffector = forwardKinematics(theta1, theta2, link1, link2);

        trajectory.push_back({endEffector.x, endEffector.y});

        theta2 += deltaTheta;
    }

    return trajectory;
}

int main()
{
    double link1 = 2.0;
    double link2 = 1.0;
    double deltaTheta = 0.01;

    std::vector<Point> trajectory = computeArmTrajectory(link1, link2, deltaTheta);

    std::vector<double> xCoords, yCoords;
    for (const auto& point : trajectory)
    {
        xCoords.push_back(point.x);
        yCoords.push_back(point.y);
    }

    plt::plot(xCoords, yCoords, "b-");
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::title("2D Robot Arm Trajectory");
    plt::show();

    return 0;
}