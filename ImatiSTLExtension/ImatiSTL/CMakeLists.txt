cmake_minimum_required(VERSION 2.8.8)

project(ImatiSTL)
set( IMATISTLLIB "${PROJECT_NAME}-lib")

set( ALGORITHMS ${IMATISTL}/src/Algorithms )
set( IMATISTL_SRC ${IMATISTL}/src/ImatiSTL )
set( KERNEL ${IMATISTL}/src/Kernel )
set( TMESH ${IMATISTL}/src/TMesh )
set( IMATISTL_H ${IMATISTL}/include/ImatiSTL )
set( KERNEL_H ${IMATISTL}/include/Kernel )
set( TMESH_H ${IMATISTL}/include/TMesh )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive -w")

# Set your files and resources here
# Set your files and resources here
file(GLOB Srcs
    "${ALGORITHMS}/*.c*"
    "${IMATISTL_SRC}/*.c*"
    "${KERNEL}/*.c*"
    "${TMESH}/*.c*"
)

file(GLOB Hdrs
    "${KERNEL_H}/*.h"
    "${TMESH_H}/*.h"
    "${IMATISTL_H}/*.h"
)

add_library(${IMATISTLLIB} ${Srcs} ${Hdrs})
target_include_directories(${IMATISTLLIB} PRIVATE
    ${IMATISTL_H}/
    ${KERNEL_H}/
    ${TMESH_H}/
)
set_target_properties(${IMATISTLLIB} PROPERTIES LINKER_LANGUAGE CXX)
