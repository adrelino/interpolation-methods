add_executable(orientation orientation.cpp)
target_link_libraries(orientation libinterpol)

add_executable(rigid-pairwise rigid-pairwise.cpp)
target_link_libraries(rigid-pairwise libinterpol)

if (EMSCRIPTEN)
    set_target_properties(orientation PROPERTIES SUFFIX ".html")
    set_target_properties(rigid-pairwise PROPERTIES SUFFIX ".html")
endif()
