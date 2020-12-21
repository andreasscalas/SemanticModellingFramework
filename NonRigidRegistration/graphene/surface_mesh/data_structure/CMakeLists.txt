
file(GLOB_RECURSE SRCS ./*.cpp ./*.c)
file(GLOB_RECURSE HDRS ./*.h)

add_library(graphene_surface_mesh STATIC ${SRCS} ${HDRS})
