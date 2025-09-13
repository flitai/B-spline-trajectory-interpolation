

cmake_minimum_required(VERSION 3.10)
project(BsplineExample)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 Eigen3 库
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# 您可以将B样条代码编译成一个库，方便在其他地方调用
add_library(bspline_lib non_uniform_bspline.cpp)
# 为库添加Eigen的头文件路径
target_include_directories(bspline_lib PUBLIC ${EIGEN3_INCLUDE_DIR})

# --- 或者，您可以直接编译成一个可执行文件（需要一个main.cpp） ---
# 示例: 创建一个 main.cpp 来测试这个库
# add_executable(bspline_test main.cpp)
# target_link_libraries(bspline_test PRIVATE bspline_lib)
# target_include_directories(bspline_test PRIVATE ${EIGEN3_INCLUDE_DIR})
