add_executable(orientation orientation.cpp)
target_link_libraries(orientation libinterpol)

add_executable(rigid-pairwise rigid-pairwise.cpp)
target_link_libraries(rigid-pairwise libinterpol)