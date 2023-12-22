cmake_minimum_required(VERSION 3.0.2)
project(cpp_exercise)

# 指定使用 C++11 标准
set(CMAKE_CXX_STANDARD 11)

# 定义变量
# 头文件目录
set(HEAD_PATH ${CMAKE_CURRENT_SOURCE_DIR}/include)

# 可执行文件生成的路径
set(EXEC_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)

# # 添加子目录
# add_subdirectory(plt_example)
# add_subdirectory(control)
add_subdirectory(robotics)