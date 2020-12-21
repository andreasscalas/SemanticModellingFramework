file(GLOB SRCS ./*.cpp)
file(GLOB HDRS ./*.h)


if (EMSCRIPTEN)

    add_library(pmp STATIC ${SRCS} ${HDRS})
    target_link_libraries(pmp imgui)

else()

    find_package(OpenGL REQUIRED)

    if (OpenGL_FOUND)
        add_library(pmp STATIC ${SRCS} ${HDRS})
        target_link_libraries(pmp imgui glfw glew stb_image ${OPENGL_LIBRARIES})
    endif()

endif()
