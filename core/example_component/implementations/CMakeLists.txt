add_library(embedded_scaffold.example.implementations ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(embedded_scaffold.example.implementations PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(embedded_scaffold.example.implementations PUBLIC
    embedded_scaffold.example.interfaces
)

target_sources(embedded_scaffold.example.implementations PRIVATE
    AccumulatorImpl.hpp
    AccumulatorImpl.cpp
)

add_subdirectory(test)
