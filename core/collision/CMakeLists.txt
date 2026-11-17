add_library(e_robot_arm.collision ${EMIL_EXCLUDE_FROM_ALL})

target_include_directories(e_robot_arm.collision PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(e_robot_arm.collision PUBLIC
    e_robot_arm.arm_model
    infra.timer
    robotics.dynamics
)

target_sources(e_robot_arm.collision PRIVATE
    CollisionObserver.cpp
    CollisionObserver.hpp
)

add_subdirectory(test)
