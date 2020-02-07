file(GLOB HEADERS
    ./*.h
    ./*.hpp
)

add_executable(vis-rigid vis-rigid.cpp ${HEADERS})
target_link_libraries(vis-rigid libinterpolviz)

add_executable(vis-orientation vis-orientation.cpp ${HEADERS})
target_link_libraries(vis-orientation libinterpolviz)


if (EMSCRIPTEN)
    set_target_properties(vis-rigid PROPERTIES SUFFIX ".html")
    target_compile_definitions(vis-rigid PUBLIC -DEMSCRIPTEN=1)
    set_target_properties(vis-rigid PROPERTIES LINK_FLAGS "-s USE_GLFW=3 -s LEGACY_GL_EMULATION=1")
    set_target_properties(vis-orientation PROPERTIES SUFFIX ".html")
    target_compile_definitions(vis-orientation PUBLIC -DEMSCRIPTEN=1)
    set_target_properties(vis-orientation PROPERTIES LINK_FLAGS "-s USE_GLFW=3 -s LEGACY_GL_EMULATION=1")
endif()
