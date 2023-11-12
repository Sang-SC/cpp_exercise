cmake_minimum_required(VERSION 3.0.2)
project(cpp_exercise)

# 指定使用 C++11 标准
set(CMAKE_CXX_STANDARD 11)

# 添加头文件搜索路径
include_directories(
    /usr/include/python3.8
    /home/ssc/cpp_exercise
)

# 添加链接库搜索路径
link_directories(
    /usr/lib/python3.8
)

# 添加可执行文件并链接库
add_executable(minimal plt_example/minimal.cpp)
target_link_libraries(minimal python3.8)

add_executable(basic plt_example/basic.cpp)
target_link_libraries(basic python3.8)

add_executable(sin plt_example/sin.cpp)
target_link_libraries(sin python3.8)