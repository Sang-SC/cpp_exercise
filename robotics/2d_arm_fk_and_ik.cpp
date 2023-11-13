#include <iostream>
#include <cmath>

struct Point
{
    double x;
    double y;
};

Point forwardKinematics(double theta1, double theta2, double link1, double link2)
{
    Point endEffector;

    endEffector.x = link1 * cos(theta1) + link2 * cos(theta1 + theta2);
    endEffector.y = link1 * sin(theta1) + link2 * sin(theta1 + theta2);

    return endEffector;
}

struct JointAngles {
    double theta1;
    double theta2;
};

JointAngles inverseKinematics(double x, double y, double link1, double link2) {
    JointAngles jointAngles;

    double c2 = (pow(x, 2) + pow(y, 2) - pow(link1, 2) - pow(link2, 2)) / (2 * link1 * link2);
    double s2 = sqrt(1 - pow(c2, 2));

    jointAngles.theta2 = atan2(s2, c2);
    jointAngles.theta1 = atan2(y, x) - atan2(link2 * s2, link1 + link2 * c2);

    return jointAngles;
}

int main()
{
    double theta1 = 1.0;
    double theta2 = 0.5;
    double link1 = 2.0;
    double link2 = 1.5;
    // 先显示初始关节角度
    std::cout << "Initial Joint Angles: theta1 = " << theta1 << ", theta2 = " << theta2 << std::endl;

    // 然后根据正运动学，求解末端位置
    Point endEffector = forwardKinematics(theta1, theta2, link1, link2);
    std::cout << "End Effector Position: (" << endEffector.x << ", " << endEffector.y << ")" << std::endl;

    // 然后根据逆运动学，反解关节角度（理论上应该和初始关节角度一致）
    JointAngles jointAngles = inverseKinematics(endEffector.x, endEffector.y, link1, link2);
    std::cout << "Joint Angles: theta1 = " << jointAngles.theta1 << ", theta2 = " << jointAngles.theta2 << std::endl;

    return 0;
}