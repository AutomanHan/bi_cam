cmake_minimum_required(VERSION 3.12)
project(bi_cam)

set(CMAKE_CXX_STANDARD 11)
set(MYNTEYE_INCLUDE_DIR /usr/local/include)
set(MYNTEYE_LIB_DIR /usr/local/lib)
set(MYNTEYE_LIB /usr/local/lib/libmynteye.so)

aux_source_directory(./ DIRSRCS)

find_package(OpenCV REQUIRED)
#find_package(PCL REQUIRED)


#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

#include_directories(${OpenCV_INCLUDE_DIRS})

#find_package(mynteye REQUIRED)
include_directories(${MYNTEYE_INCLUDE_DIR})
include_directories("./util")
link_directories(${MYNTEYE_LIB_DIR})
#link_libraries()

add_executable(bi_cam ${DIRSRCS})

#link_directories(${opencv_library_dirs})
#link_directories(${mynteye_library_dirs})

target_link_libraries(bi_cam ${OpenCV_LIBS})
target_link_libraries(bi_cam ${MYNTEYE_LIB})

#target_link_libraries (bi_cam ${PCL_LIBRARIES})

#install(TARGETS bi_cam RUNTIME DESTINATION bin)