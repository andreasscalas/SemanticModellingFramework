cmake_minimum_required(VERSION 3.1)
project(ShapeOp)

# Tell CMake he'll be able to include ".cmake" configurations 
# files in the folder where the current file is located
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_ENABLE_EXTENDED_ALIGNED_STORAGE")
# Find Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
if(NOT EIGEN3_FOUND)
    message(ERROR " EIGEN not found.")
else()
    message("EIGEN found.")
endif() 

# Where are the headers found?
    include_directories("src/")
    include_directories("api/")

# h files
file(GLOB SHAPEOP_HEADERS "src/*.h")
# cpp files
file(GLOB SHAPEOP_SOURCES "src/*.cpp")


set( IMATISTL ${MAINFOLDER}/ImatiSTLExtension/ImatiSTL )
set( IMATISTL_H ${IMATISTL}/include/ImatiSTL )
set( KERNEL_H ${IMATISTL}/include/Kernel )
set( TMESH_H ${IMATISTL}/include/TMesh )
include_directories(
    ${IMATISTL_H}/
    ${KERNEL_H}/
    ${TMESH_H}/
    ${IMATISTLEX_H}/
    ${NANOFLANN}/include/
    ${NANOFLANN}/examples/
    ${EIGEN_H}
    ${ANNOTATION_H}
    ${DATA_STRUCTURES_H}/
    ${ORTOOLS_H}/
)

# Shared library
set(HEADERS ${SHAPEOP_HEADERS} ${API_HEADERS})
set(SOURCES ${SHAPEOP_SOURCES} ${API_SOURCES})
add_library(ShapeOp SHARED ${HEADERS} ${SOURCES})
set_target_properties(ShapeOp PROPERTIES VERSION ${SHAPEOP_VERSION_STRING} SOVERSION ${SHAPEOP_VERSION_MAJOR})


# Rename executable to contain version number on windows. On mac, a so-chain is generated from the SOVERSION specified above.
if(CMAKE_SYSTEM_NAME MATCHES "Windows")
	set_target_properties(ShapeOp PROPERTIES OUTPUT_NAME ShapeOp.${SHAPEOP_VERSION_STRING})


endif()

add_dependencies(ShapeOp ImatiSTLExtension-lib ImatiSTL-lib DataStructures-lib)
target_include_directories(ShapeOp PRIVATE
    ${IMATISTL_H}/
    ${KERNEL_H}/
    ${TMESH_H}/
    ${IMATISTLEX_H}/
    ${EIGEN_H}
    ${NANOFLANN}/include/
    ${NANOFLANN}/examples/
    ${ANNOTATION_H}
    ${DATA_STRUCTURES_H}/
)
#Adding subdirectories
