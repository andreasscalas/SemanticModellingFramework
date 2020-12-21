cmake_minimum_required(VERSION 2.8)

project(Deformation)

find_package(VTK REQUIRED)

find_package(VTK COMPONENTS
  vtkCommonCore
  vtkFiltersSources
  vtkInteractionStyle
  vtkInteractionWidgets
  vtkRendering${VTK_RENDERING_BACKEND}
)
include(${VTK_USE_FILE})

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/../
    ${CMAKE_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../ImatiSTL-4.2/include/ImatiSTL
    ${CMAKE_CURRENT_SOURCE_DIR}/../../ImatiSTL-4.2/include/Kernel
    ${CMAKE_CURRENT_SOURCE_DIR}/../../ImatiSTL-4.2/include/TMesh
    ${CMAKE_CURRENT_SOURCE_DIR}/../../ImatiSTL-4.2/include/Rendering
)

set( ALGORITHMS ${CMAKE_SOURCE_DIR}/../../ImatiSTL-4.2/src/Algorithms )
set( IMATISTL ${CMAKE_SOURCE_DIR}/../../ImatiSTL-4.2/src/ImatiSTL )
set( KERNEL ${CMAKE_SOURCE_DIR}/../../ImatiSTL-4.2/src/Kernel )
set( TMESH ${CMAKE_SOURCE_DIR}/../../ImatiSTL-4.2/src/TMesh )
set( RENDERING ${CMAKE_SOURCE_DIR}/../../ImatiSTL-4.2/src/Rendering)
set( IMATISTL_H ${CMAKE_SOURCE_DIR}/../../ImatiSTL-4.2/include/ImatiSTL )
set( KERNEL_H ${CMAKE_SOURCE_DIR}/../../ImatiSTL-4.2/include/Kernel )
set( TMESH_H ${CMAKE_SOURCE_DIR}/../../ImatiSTL-4.2/include/TMesh )
set( RENDERING_H ${CMAKE_SOURCE_DIR}/../../ImatiSTL-4.2/include/Rendering)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive -w")

# Set your files and resources here
set( Srcs
  ${CMAKE_SOURCE_DIR}/../src/meanvaluecoordinates.cpp
  ${ALGORITHMS}/checkAndRepair.cpp
  ${ALGORITHMS}/delaunize.cpp
  ${ALGORITHMS}/detectIntersections.cpp
  ${ALGORITHMS}/faceClustering.cpp
  ${ALGORITHMS}/featureRecover.cpp
  ${ALGORITHMS}/holeFilling.cpp
  ${ALGORITHMS}/simplification.cpp
  ${ALGORITHMS}/smoothing.cpp
  ${ALGORITHMS}/spherize.cpp
  ${ALGORITHMS}/subdivision.cpp
  ${IMATISTL}/computeOuterHull.cpp
  ${IMATISTL}/cutIntersections.cpp
  ${IMATISTL}/dijkstraGraph.cpp
  ${IMATISTL}/epsilonSampling.cpp
  ${IMATISTL}/marchIntersections.cpp
  ${IMATISTL}/offset.cpp
  ${IMATISTL}/uniformRemesh.cpp
  ${KERNEL}/coordinates.cpp
  ${KERNEL}/graph.cpp
  ${KERNEL}/heap.cpp
  ${KERNEL}/jqsort.cpp
  ${KERNEL}/list.cpp
  ${KERNEL}/matrix.cpp
  ${KERNEL}/orientation.c
  ${KERNEL}/point.cpp
  ${KERNEL}/tmesh.cpp
  ${TMESH}/edge.cpp
  ${TMESH}/edgeHeap.cpp
  ${TMESH}/io.cpp
  ${TMESH}/tin.cpp
  ${TMESH}/triangle.cpp
  ${TMESH}/vertex.cpp
  ${RENDERING}/drawablemesh.cpp
)

set( Hdrs
    ${CMAKE_SOURCE_DIR}/../include/meanvaluecoordinates.h
    ${CMAKE_SOURCE_DIR}/../include/barycentriccoordinates.h
    ${IMATISTL_H}/imatistl.h
    ${IMATISTL_H}/trimesh.h
    ${IMATISTL_H}/dijkstraGraph.h
    ${IMATISTL_H}/marchIntersections.h
    ${IMATISTL_H}/cutIntersections.h
    ${KERNEL_H}/basics.h
    ${KERNEL_H}/graph.h
    ${KERNEL_H}/jqsort.h
    ${KERNEL_H}/matrix.h
    ${KERNEL_H}/tmesh_kernel.h
    ${KERNEL_H}/coordinates.h
    ${KERNEL_H}/heap.h
    ${KERNEL_H}/list.h
    ${KERNEL_H}/point.h
    ${TMESH_H}/detectIntersections.h
    ${TMESH_H}/edgeHeap.h
    ${TMESH_H}/simplification.h
    ${TMESH_H}/tmesh.h
    ${TMESH_H}/vertex.h
    ${TMESH_H}/edge.h
    ${TMESH_H}/faceClustering.h
    ${TMESH_H}/tin.h
    ${TMESH_H}/triangle.h
    ${RENDERING_H}/drawablemesh.h
)

add_library(deformation ${Srcs} ${Hdrs})

target_link_libraries(deformation ${VTK_LIBRARIES})
