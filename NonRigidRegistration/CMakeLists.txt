project(characterfitting)


cmake_minimum_required(VERSION 3.1)


set(CMAKE_CXX_STANDARD 11)
set(PROJECT_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})

# Dependencies
find_package(OpenGL REQUIRED)
find_package(OpenMP)
find_package(OpenCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(FBX QUIET)
find_package(dlib REQUIRED)



if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()


##############################################################################
# GLFW
##############################################################################
if (NOT EMSCRIPTEN)

    set(GLFW_SOURCE_DIR  "external/glfw")

    set(BUILD_SHARED_LIBS OFF CACHE BOOL "")
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "")
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "")
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "")
    set(GLFW_INSTALL OFF CACHE BOOL "")

    add_subdirectory(${GLFW_SOURCE_DIR})

    include_directories(${GLFW_SOURCE_DIR}/include
                        ${GLFW_SOURCE_DIR}/deps)
else()

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -s USE_GLFW=3")

endif()
##############################################################################


##############################################################################
# GLEW
##############################################################################
if (NOT EMSCRIPTEN)

    set(GLEW_SOURCE_DIR  "external/glew")

    include_directories(${GLEW_SOURCE_DIR}/include)

    add_definitions(-DGLEW_STATIC)
    add_library(glew STATIC
                ${GLEW_SOURCE_DIR}/src/glew.c
                ${GLEW_SOURCE_DIR}/include)
    target_link_libraries(glew ${GLFW_LIBRARIES})

endif()
##############################################################################


##############################################################################
# imgui
##############################################################################
set(IMGUI_SOURCE_DIR "external/imgui")
include_directories(${IMGUI_SOURCE_DIR})
add_subdirectory(${IMGUI_SOURCE_DIR})
##############################################################################


##############################################################################
# pmp
##############################################################################
set(PMP_SOURCE_DIR "external/pmp")
include_directories(external)
add_subdirectory(external/pmp)
##############################################################################



##############################################################################
# stb_image
##############################################################################
add_subdirectory(external/stb_image)
##############################################################################


##############################################################################
# tiny xml
##############################################################################
add_definitions(-DTIXML_USE_STL)
add_subdirectory(external/tinyxml)
##############################################################################


##############################################################################
# opencl
##############################################################################
if(OpenCL_FOUND)
    add_definitions(-DHAVE_OCL)
    include_directories(external/opencl/include)
    add_subdirectory(external/opencl)
endif()
##############################################################################


##############################################################################
# OpenMP
##############################################################################
if(OpenMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  if(MSVC)
    add_definitions(/arch:SSE)
  endif(MSVC)
  set(CMAKE_SHARED_LINKER_FLAGS ${CMAKE_SHARED_LINKER_FLAGS_INIT} -fopenmp)
endif()
##############################################################################




##############################################################################
# fbxsdk
##############################################################################
include_directories(external/fbxsdk/include)
find_file(FBX_LIBRARY libfbxsdk.a ./external/fbxsdk/lib/)
find_file(FBX_INCLUDE fbxsdk.h ./external/fbxsdk/include/)
if (NOT FBX_LIBRARY OR NOT FBX_INCLUDE)
    message(WARNING "FBXSDK NOT found in external/fbxsdk/. Please make sure to copy libfbxsdk.a in external/fbxsdk/lib/ and the header files to external/fbxsdk/include/ if you want to write fbx files!")
else()
    add_definitions(-DHAVE_FBX)
endif()
##############################################################################



##############################################################################
# source files
##############################################################################
include_directories(${NONRIGID_REGISTRATION}/src/)
add_subdirectory(src)
##############################################################################


##############################################################################
# copy data folder
##############################################################################
file(COPY ./data/ DESTINATION ${PROJECT_BINARY_DIR})
##############################################################################
