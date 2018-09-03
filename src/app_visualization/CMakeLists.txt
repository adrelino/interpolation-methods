file(GLOB HEADERS
    ./*.h
    ./*.hpp
)

add_executable(vis-rigid vis-rigid.cpp ${HEADERS})
target_link_libraries(vis-rigid libinterpolviz)

add_executable(vis-orientation vis-orientation.cpp ${HEADERS})
target_link_libraries(vis-orientation libinterpolviz)
