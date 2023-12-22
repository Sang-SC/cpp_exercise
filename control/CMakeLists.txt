cmake_minimum_required(VERSION 3.0.2)
project(control)

# 指定使用 C++11 标准
set(CMAKE_CXX_STANDARD 11)

# 指定可执行文件的输出路径
set(EXECUTABLE_OUTPUT_PATH ${EXEC_PATH})

include_directories(
    ${HEAD_PATH}  # 变量HEAD_PATH在根节点的 CMakeLists.txt 中定义
    /usr/include/python3.10  # matplotlibcpp 需要用到相关头文件
)

# 可执行文件
add_executable(PID src/PID.cpp)
target_link_libraries(PID python3.10)