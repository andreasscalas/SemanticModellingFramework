
file(GLOB_RECURSE SRCS ./*.h ./*.cpp)

add_library(graphene_character_scene_graph STATIC ${SRCS} )

target_link_libraries(graphene_character_scene_graph
 graphene_surface_mesh_scene_graph
 graphene_scene_graph 
 graphene_character)
