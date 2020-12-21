include_directories(${CMAKE_SOURCE_DIR}/src/)

file(GLOB_RECURSE SRCS ./*.cpp)
file(GLOB_RECURSE HDRS ./*.h)

add_library(graphene_surface_mesh_scene_graph STATIC ${SRCS} ${HDRS})

target_link_libraries(graphene_surface_mesh_scene_graph
  graphene_scene_graph
  graphene_surface_mesh)
