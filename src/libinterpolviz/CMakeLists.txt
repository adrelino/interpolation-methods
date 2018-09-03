find_package(OpenGL REQUIRED)
find_package(glfw3 3.2 REQUIRED)
message("glfw found with version " ${glfw3_VERSION})

file(GLOB ALL
    include/interpolviz/*.h
    include/interpolviz/utils/*.h
    src/*.cpp
)

file(GLOB TRAJECTORY
    include/interpolviz/trajectory/*.h
    src/trajectory/*.cpp
)

add_library(libinterpolviz STATIC ${ALL} ${TRAJECTORY})
target_include_directories(libinterpolviz PUBLIC include/)
target_include_directories(libinterpolviz PRIVATE include/interpolviz/)
target_include_directories(libinterpolviz PRIVATE include/interpolviz/trajectory)

target_link_libraries(libinterpolviz libinterpol glfw ${OPENGL_gl_LIBRARY})
