pendulum_dynamics.cpp 文件仿真时，给初始角度自然摆落，由于无阻尼，按理应该振荡循环，但发现振荡越来越大，一开始以为动力学部分问题，后来发现其实是步长问题，0.01 振荡会越来越大，0.001 则基本不会