#include <iostream>
#include <vector>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main() {
    // 创建一个图形窗口
    plt::figure();

    // 设置绘图参数
    int numFrames = 100;  // 动画帧数
    std::vector<double> x(numFrames);  // x坐标
    std::vector<double> y(numFrames);  // y坐标

    // 生成动画数据
    for (int i = 0; i < numFrames; ++i) {
        double t = 0.1 * i;
        x[i] = std::sin(t);
        y[i] = std::cos(t);
    }

    // 绘制动画
    for (int i = 0; i < numFrames; ++i) {
        // 清除当前图形
        plt::clf();

        // 绘制当前帧
        plt::plot({x[i]}, {y[i]}, "ro");

        // 设置坐标轴范围
        plt::xlim(-1.2, 1.2);
        plt::ylim(-1.2, 1.2);

        // 显示绘图
        plt::draw();
        plt::pause(0.01);
    }

    // 关闭图形窗口
    plt::close();

    return 0;
}