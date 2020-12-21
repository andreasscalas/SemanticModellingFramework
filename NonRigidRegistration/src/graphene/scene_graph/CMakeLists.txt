
file(GLOB_RECURSE SRCS ./*.cpp ./*.c ./*.h)

add_library(graphene_scene_graph STATIC ${SRCS})

target_link_libraries(graphene_scene_graph
  graphene_geometry
  graphene_gl
)
