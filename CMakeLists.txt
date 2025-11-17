CMAKE_MINIMUM_REQUIRED(VERSION 3.16)
PROJECT(ZED)

# set the LANGUAGE
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-12.8)
 

set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

find_package(ZED 3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
find_package(Threads REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common io visualization)
 

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)
 

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 

FILE(GLOB_RECURSE SRC_FILES src/*.cpp  src/Zed_cuda.cu)
FILE(GLOB_RECURSE HDR_FILES include/*.hpp include/*.h)

CUDA_ADD_EXECUTABLE(run main.cpp ${HDR_FILES} ${SRC_FILES})

target_link_libraries(run ${ZED_LIBRARIES})
target_link_libraries(run ${OpenCV_LIBS})
target_link_libraries(run Threads::Threads)
target_link_libraries(run ${PCL_LIBRARIES})

CUDA_ADD_EXECUTABLE(getpcd getpcd.cpp ${HDR_FILES} ${SRC_FILES})

target_link_libraries(getpcd ${ZED_LIBRARIES})
target_link_libraries(getpcd ${OpenCV_LIBS})
target_link_libraries(getpcd Threads::Threads)
target_link_libraries(getpcd ${PCL_LIBRARIES})

