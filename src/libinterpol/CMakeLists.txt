set(INTERPOL_HEADERS
    include/interpol/euclidean.hpp
    include/interpol/orientation.hpp
    include/interpol/utils/timer.hpp
    include/interpol/utils/random.hpp
)

set(INTERPOL_SOURCES
    src/utils/timer.cpp
    src/utils/random.cpp
)

add_library(libinterpol STATIC ${INTERPOL_HEADERS} ${INTERPOL_SOURCES})
target_include_directories(libinterpol PUBLIC include/)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(libinterpol Eigen3::Eigen)
