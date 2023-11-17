cmake_minimum_required(VERSION 3.0.2)
project(cpp_exercise)

# 指定使用 C++11 标准
set(CMAKE_CXX_STANDARD 11)

# 添加头文件搜索路径
include_directories(
    /usr/include/python3.8
    /home/ssc/cpp_exercise
    /usr/include/eigen3
)

# 添加链接库搜索路径
link_directories(
    /usr/lib/python3.8
)

# 添加可执行文件并链接库
# ------------------------plt_example------------------------
add_executable(minimal plt_example/minimal.cpp)
target_link_libraries(minimal python3.8)

add_executable(basic plt_example/basic.cpp)
target_link_libraries(basic python3.8)

add_executable(sin plt_example/sin.cpp)
target_link_libraries(sin python3.8)

add_executable(subplot plt_example/subplot.cpp)
target_link_libraries(subplot python3.8)

add_executable(animation plt_example/animation.cpp)
target_link_libraries(animation python3.8)

# ------------------------control------------------------
add_executable(PID control/PID.cpp)
target_link_libraries(PID python3.8)

# ------------------------robotics------------------------
add_executable(2d_arm_fk_and_ik robotics/2d_arm_fk_and_ik.cpp)

add_executable(2d_arm_trajectory robotics/2d_arm_trajectory.cpp)
target_link_libraries(2d_arm_trajectory python3.8)

add_executable(3d_RRR_kinematics robotics/3d_RRR_kinematics.cpp)
target_link_libraries(3d_RRR_kinematics python3.8)

add_executable(pendulum_dynamics robotics/pendulum_dynamics.cpp)
target_link_libraries(pendulum_dynamics python3.8)

add_executable(pendulum_dynamics2 robotics/pendulum_dynamics2.cpp)
target_link_libraries(pendulum_dynamics2 python3.8)