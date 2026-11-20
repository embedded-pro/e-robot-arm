include(GNUInstallDirs)

set(component tools)
set(resources "${CMAKE_CURRENT_LIST_DIR}/resources")
set(E_ROBOT_ARM_PACKAGE_ENGINE "" CACHE FILEPATH "e-foc-virtual-motor to ship in the tools package")

foreach(program e_robot_arm.tool.simulator e_robot_arm.tool.arm_console e_robot_arm.tool.virtual_arm.main)
    if (TARGET ${program})
        install(TARGETS ${program} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT ${component} EXCLUDE_FROM_ALL)
    endif()
endforeach()

if (E_ROBOT_ARM_PACKAGE_ENGINE)
    install(PROGRAMS "${E_ROBOT_ARM_PACKAGE_ENGINE}" DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT ${component} EXCLUDE_FROM_ALL)
endif()

if (WIN32)
    install(FILES "${PROJECT_SOURCE_DIR}/LICENSE" DESTINATION . RENAME LICENSE.txt COMPONENT ${component} EXCLUDE_FROM_ALL)

    if (NOT CMAKE_CROSSCOMPILING AND TARGET e_robot_arm.tool.simulator)
        find_program(E_ROBOT_ARM_WINDEPLOYQT windeployqt HINTS "${QT6_INSTALL_PREFIX}/bin" REQUIRED)
        set(qml_sources "${PROJECT_SOURCE_DIR}/tools/simulator/app/qml")
        set(ui_qml_sources "${PROJECT_SOURCE_DIR}/infra/ui-cpp/ui/backend/qt/quick3d")
        install(CODE "
            execute_process(
                COMMAND \"${E_ROBOT_ARM_WINDEPLOYQT}\" --release --no-translations
                    --qmldir \"${qml_sources}\" --qmldir \"${ui_qml_sources}\"
                    \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}/e-robot-arm-simulator.exe\"
                COMMAND_ERROR_IS_FATAL ANY)
            " COMPONENT ${component} EXCLUDE_FROM_ALL)
    endif()
else()
    install(FILES e-robot-arm-simulator.desktop DESTINATION ${CMAKE_INSTALL_DATADIR}/applications COMPONENT ${component} EXCLUDE_FROM_ALL)
    install(FILES "${resources}/e-robot-arm-simulator.svg" DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/scalable/apps COMPONENT ${component} EXCLUDE_FROM_ALL)
    install(FILES "${resources}/e-robot-arm-simulator.png" DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/256x256/apps COMPONENT ${component} EXCLUDE_FROM_ALL)
    install(FILES "${PROJECT_SOURCE_DIR}/LICENSE" DESTINATION ${CMAKE_INSTALL_DATADIR}/doc/e-robot-arm RENAME copyright COMPONENT ${component} EXCLUDE_FROM_ALL)
endif()

function(e_robot_arm_tools_package)
    set(CPACK_OUTPUT_CONFIG_FILE "${CMAKE_CURRENT_BINARY_DIR}/CPackConfig.cmake")
    set(CPACK_SOURCE_OUTPUT_CONFIG_FILE "${CMAKE_CURRENT_BINARY_DIR}/CPackSourceConfig.cmake")
    set(CPACK_PACKAGE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/packages")
    set(CPACK_INSTALL_CMAKE_PROJECTS "${CMAKE_BINARY_DIR};e_robot_arm;${component};/")
    set(CPACK_MONOLITHIC_INSTALL ON)
    set(CPACK_VERBATIM_VARIABLES ON)
    set(CPACK_GENERATOR "DEB;NSIS")

    set(CPACK_PACKAGE_NAME "e-robot-arm")
    set(CPACK_PACKAGE_VENDOR "embedded-pro")
    set(CPACK_PACKAGE_VERSION "${e_robot_arm_VERSION}")
    set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "e-robot-arm Simulator and tools for a six-joint arm on e-foc motor nodes")
    set(CPACK_PACKAGE_HOMEPAGE_URL "https://github.com/embedded-pro/e-robot-arm")
    set(CPACK_PACKAGE_FILE_NAME "e_robot_arm-tools-v${e_robot_arm_VERSION}-${CMAKE_SYSTEM_NAME}")
    set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/LICENSE")
    set(CPACK_STRIP_FILES ON)

    set(CPACK_DEBIAN_PACKAGE_NAME "e-robot-arm")
    set(CPACK_DEBIAN_PACKAGE_MAINTAINER "embedded-pro")
    set(CPACK_DEBIAN_PACKAGE_SECTION "electronics")
    set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
    set(CPACK_DEBIAN_PACKAGE_DEPENDS
        "qt6-qpa-plugins, qml6-module-qtqml, qml6-module-qtqml-workerscript, qml6-module-qtquick, qml6-module-qtquick-window, qml6-module-qtquick3d | qml6-module-quick3d, qml6-module-qtquick3d-assetutils | qml6-module-quick3d-assetutils, qml6-module-qtquick3d-helpers | qml6-module-quick3d-helpers")
    set(CPACK_DEBIAN_PACKAGE_DESCRIPTION
        "Simulator and console for a six-joint robot arm on e-foc motor nodes.\n Drives the arm on simulated joints, on e-foc virtual motors (included) or on\n the real arm through a CAN adapter.")

    set(CPACK_PACKAGE_EXECUTABLES "e-robot-arm-simulator" "e-robot-arm Simulator")
    set(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES64")
    set(CPACK_PACKAGE_INSTALL_DIRECTORY "e-robot-arm")
    set(CPACK_PACKAGE_INSTALL_REGISTRY_KEY "e-robot-arm")
    set(CPACK_NSIS_PACKAGE_NAME "e-robot-arm")
    set(CPACK_NSIS_DISPLAY_NAME "e-robot-arm")
    set(CPACK_NSIS_BRANDING_TEXT "e-robot-arm")
    set(CPACK_NSIS_MUI_ICON "${resources}/e-robot-arm-simulator.ico")
    set(CPACK_NSIS_MUI_UNIICON "${resources}/e-robot-arm-simulator.ico")
    set(CPACK_NSIS_INSTALLED_ICON_NAME "bin\\e-robot-arm-simulator.exe")
    set(CPACK_NSIS_MUI_FINISHPAGE_RUN "e-robot-arm-simulator.exe")
    set(CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL ON)
    set(CPACK_NSIS_MANIFEST_DPI_AWARE ON)
    set(CPACK_NSIS_URL_INFO_ABOUT "https://github.com/embedded-pro/e-robot-arm")
    set(CPACK_NSIS_HELP_LINK "https://github.com/embedded-pro/e-robot-arm/issues")
    set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS [=[
  CreateShortCut "$DESKTOP\e-robot-arm Simulator.lnk" "$INSTDIR\bin\e-robot-arm-simulator.exe"
  IfFileExists "$INSTDIR\bin\vc_redist.x64.exe" 0 +3
    ExecWait '"$INSTDIR\bin\vc_redist.x64.exe" /install /quiet /norestart'
    Delete "$INSTDIR\bin\vc_redist.x64.exe"
]=])
    set(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS [=[
  Delete "$DESKTOP\e-robot-arm Simulator.lnk"
]=])

    include(CPack)
endfunction()

e_robot_arm_tools_package()
