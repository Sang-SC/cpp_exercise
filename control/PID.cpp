#include <iostream>
#include <vector>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

class PIDController {
private:
    double Kp;
    double Ki;
    double Kd;

    double target;            // 目标位置
    double integral;          // 积分累计
    double prevError;         // 历史误差

public:
    PIDController(double kp, double ki, double kd, double target) : Kp(kp), Ki(ki), Kd(kd), target(target), integral(0), prevError(0) {};
    double compute(double input, double dt)
    {
        double error = target - input;
        double P = Kp * error;            // 比例项

        integral += error * dt;
        double I = Ki * integral;         // 积分项

        double derivative = (error - prevError) / dt;
        double D = Kd * derivative;       // 微分项

        double output = P + I + D;

        prevError = error;                // 更新误差

        return output;
    }
};

int main()
{
    // 设置 PID 参数
    double kp = 2;
    double ki = 10;
    double kd = 0;

    // 设置目标位置、当前位置和dt
    double target = 10.0;
    double input = 0;
    double dt = 0.01;

    PIDController controller(kp, ki, kd, target);

    std::vector<double> time;
    std::vector<double> inputs;
    std::vector<double> outputs;


    for (int i = 0; i < 1000; ++i) 
    {
        double output = controller.compute(input, dt);

        input += output * dt;

        time.push_back(i * dt);
        inputs.push_back(input);
        outputs.push_back(output);

        std::cout << "Iteration: " << i << ", Input: " << input << ", Output: " << output << std::endl;
    }

    plt::plot(time, inputs);
    plt::xlabel("Time/s");
    plt::ylabel("Value");
    plt::show();

    return 0;
}

