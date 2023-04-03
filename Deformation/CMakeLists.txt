cmake_minimum_required(VERSION 2.8.8)

project(Deformation)
set( DEFORMATIONLIB "${PROJECT_NAME}-lib")
set( DEFORMATION_SRC ${DEFORMATION}/src)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive -w")


file(GLOB Srcs "${DEFORMATION_SRC}/*.c*")
file(GLOB Hdrs "${DEFORMATION_H}/*.h" )
# Set your files and resources here


add_library(${DEFORMATIONLIB} ${Srcs} ${Hdrs})
qt5_use_modules(${DEFORMATIONLIB} Core Gui Widgets)
target_include_directories(${DEFORMATIONLIB} PRIVATE
    ${DEFORMATION_H}/
    ${MAINFOLDER}/include
    ${MAINFOLDER}/Styles/include
    ${IMATISTLEX_H}
    ${IMATISTL_H}/ImatiSTL/
    ${IMATISTL_H}/Kernel/
    ${IMATISTL_H}/TMesh/
    ${ANNOTATION_H}
    ${SLICER_H}
    ${NANOFLANN}/include/
    ${NANOFLANN}/examples/
    ${EIGEN_H}
    ${SHAPEOP}/src
    ${ORTOOLS_H}/
)

target_link_libraries(${DEFORMATIONLIB} ImatiSTLExtension-lib)
