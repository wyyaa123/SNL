# CMakeList.txt: test 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.8)
project ("test")

set(CMAKE_TOOLCHAIN_FILE "$E:/src/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")
include_directories(
    ./include
)

# 将源代码添加到此项目的可执行文件。
add_executable (main src/main.cpp )

find_package(OpenCV REQUIRED)
target_include_directories(main PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(main PRIVATE ${OpenCV_LIBS})

# TODO: 如有需要，请添加测试并安装目标。
