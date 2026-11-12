add_subdirectory(view_models)

if (E_ROBOT_ARM_BUILD_SIMULATOR)
    add_subdirectory(app)
endif()
