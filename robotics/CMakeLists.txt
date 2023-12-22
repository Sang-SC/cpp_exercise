cmake_minimum_required(VERSION 3.0.2)
project(robotics)

# 指定使用 C++11 标准
set(CMAKE_CXX_STANDARD 11)

# 指定可执行文件的输出路径
set(EXECUTABLE_OUTPUT_PATH ${EXEC_PATH})

include_directories(
    ${HEAD_PATH}  # 变量HEAD_PATH在根节点的 CMakeLists.txt 中定义
    /usr/include/python3.10  # matplotlibcpp 需要用到相关头文件
    /usr/include/eigen3
)

# 可执行文件
add_executable(2d_arm_fk_and_ik src/2d_arm_fk_and_ik.cpp)

add_executable(2d_arm_trajectory src/2d_arm_trajectory.cpp)
target_link_libraries(2d_arm_trajectory python3.10)

add_executable(3d_RRR_kinematics src/3d_RRR_kinematics.cpp)

add_executable(pendulum_dynamics src/pendulum_dynamics.cpp)
target_link_libraries(pendulum_dynamics python3.10)