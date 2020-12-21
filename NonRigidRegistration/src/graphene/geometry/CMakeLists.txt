include_directories(${NONRIGID_REGISTRATION}/src/)

file(GLOB_RECURSE SRCS ./*.cpp)
file(GLOB_RECURSE HDRS ./*.h)

add_library(graphene_geometry STATIC ${SRCS} ${HDRS})

