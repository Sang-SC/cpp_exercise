#include <iostream>
#include <cmath>
#include <vector>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

const double g = 9.81;

struct State {
    double theta;
    double omega;
};

State pendulumDynamics(const State& state, double dt, double pendulumLength)
{
    State state_new;

    double accel = g / pendulumLength * sin(state.theta) - state.omega * 0.5;  // 角加速度计算公式，一开始以为没有pendulumLength，后来稍微推了一下发现确实得有

    state_new.theta = state.theta + state.omega * dt;
    state_new.omega = state.omega + accel * dt;

    return state_new;
}

std::vector<State> simulatePendulum(double theta0, double omega0, double dt, double simulationTime, double pendulumLength)
{
    std::vector<State> trajectory;

    double t = 0.0;

    State currentState = {theta0, omega0};

    while (t <= simulationTime)
    {
        trajectory.push_back(currentState);
        currentState = pendulumDynamics(currentState, dt, pendulumLength);

        plt::clf();

        std::vector<double> x;
        std::vector<double> y;
        x.push_back(0);
        x.push_back(pendulumLength * cos(currentState.theta));
        y.push_back(0);
        y.push_back(pendulumLength * sin(currentState.theta));
        
        plt::plot(x, y);
        plt::title("Inverted Pendulum Animation");

        plt::pause(0.001);

        t += dt;
    }
    return trajectory;
}

// function: add two double number
double add(double a, double b)
{
    return a + b;
}




int main()
{
    double theta0 = M_PI * 2 / 6;
    double omega0 = 0.0;
    double dt = 0.001;  // 这里的步长影响很大！0.01则发现振荡越来越大，不合理，一开始以为动力学部分问题，后来发现其实是步长问题
    double simulationTime = 10.0;
    double pendulumLength = 1.0;

    std::vector<State> trajectory = simulatePendulum(theta0, omega0, dt, simulationTime, pendulumLength);


    // std::vector<double> time, theta;
    // for (const auto& state : trajectory)
    // {
    //     time.push_back(dt * time.size());
    //     theta.push_back(state.theta);
    // }

    // plt::plot(time, theta, "b-");
    // plt::xlabel("Time (s)");
    // plt::ylabel("Angular Position (rad)");
    // plt::title("Pendulum Dynamics");
    // plt::show();

    return 0;
}