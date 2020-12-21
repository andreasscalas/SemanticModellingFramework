file(GLOB SRCS ./*.h ./*.cpp )


add_library(io STATIC ${SRCS})
