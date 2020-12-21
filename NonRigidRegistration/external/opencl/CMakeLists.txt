file(GLOB SRCS ./*.cpp)
file(GLOB HDRS ./*.h)

if(OpenCL_FOUND)
    add_library(cl STATIC ${SRCS} ${HDRS})
    target_link_libraries(cl ${OpenCL_LIBRARY})
    file(COPY kernels DESTINATION ${PROJECT_BINARY_DIR})
endif()


