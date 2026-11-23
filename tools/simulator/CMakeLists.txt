add_subdirectory(view_models)
add_subdirectory(samples)

if (E_ROBOT_ARM_BUILD_SIMULATOR)
    add_subdirectory(app)
endif()
