# CMakeList.txt: test 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.8)
project ("test")

#set(CMAKE_TOOLCHAIN_FILE "$E:/src/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")

include_directories(
    ./include
)

# 将源代码添加到此项目的可执行文件。
#add_executable (FeatPoint src/FeatPoint.cpp )

add_executable (BlurKern src/BlurKern.cpp )

#add_executable (KalFilter src/KalFilter.cpp )

set(OpenCV_DIR "${VCPKG_INSTALLED_DIR}/x64-windows/share/opencv3")
find_package(OpenCV REQUIRED)
#target_include_directories(FeatPoint PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(BlurKern PRIVATE ${OpenCV_INCLUDE_DIRS})
#target_include_directories(KalFilter PRIVATE ${OpenCV_INCLUDE_DIRS})
#target_link_libraries(FeatPoint PRIVATE ${OpenCV_LIBS})
target_link_libraries(BlurKern PRIVATE ${OpenCV_LIBS})
#target_link_libraries(KalFilter PRIVATE ${OpenCV_LIBS})

find_package(Eigen3 REQUIRED)
#target_include_directories(FeatPoint PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(BlurKern PRIVATE ${Eigen3_INCLUDE_DIRS})
#target_include_directories(KalFilter PRIVATE ${Eigen3_INCLUDE_DIRS})
#target_link_libraries(FeatPoint PRIVATE ${Eigen3_LIBS})
target_link_libraries(BlurKern PRIVATE ${Eigen3_LIBS})
#target_link_libraries(KalFilter PRIVATE ${Eigen3_LIBS})

find_package(Ceres CONFIG REQUIRED)
#target_link_libraries(FeatPoint PRIVATE Ceres::ceres)
target_link_libraries(BlurKern PRIVATE Ceres::ceres)
#target_link_libraries(KalFilter PRIVATE Ceres::ceres)

# TODO: 如有需要，请添加测试并安装目标。
