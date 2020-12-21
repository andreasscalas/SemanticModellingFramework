cmake_minimum_required(VERSION 2.8.8)
project(ConstraintsLib)
set( CONSTRAINTSLIB "${PROJECT_NAME}-lib")
set( CONSTRAINTS_SRC ${MAINFOLDER}/Constraints/src/)
set( CONSTRAINTS_H ${MAINFOLDER}/Constraints/include/)

file(GLOB Srcs "${CONSTRAINTS_SRC}/*.c*")
file(GLOB Hdrs "${CONSTRAINTS_H}/*.h")
add_library(${CONSTRAINTSLIB} ${Srcs} ${Hdrs})
target_include_directories(${CONSTRAINTSLIB} PRIVATE
    ${CONSTRAINTS_H}/
    ${MAINFOLDER}/ImatiSTLExtension/include/
    ${IMATISTL_H}/ImatiSTL/
    ${IMATISTL_H}/Kernel/
    ${IMATISTL_H}/TMesh/
    ${SLICER_H}
    ${MAINFOLDER}/include/
    ${EIGEN_H}
)

target_link_libraries(${CONSTRAINTSLIB} ImatiSTLExtension-lib)
add_executable(${PROJECT_NAME} ${Srcs} ${Hdrs})
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CONSTRAINTS_H}/
    ${MAINFOLDER}/ImatiSTLExtension/include/
    ${IMATISTL_H}/ImatiSTL/
    ${IMATISTL_H}/Kernel/
    ${IMATISTL_H}/TMesh/
    ${SLICER_H}
    ${MAINFOLDER}/include/
    ${EIGEN_H}
)

