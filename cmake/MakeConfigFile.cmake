# Writes a config file
set(CONFIG_TEMPLATE_PATH ${CMAKE_CURRENT_LIST_DIR})

function(MakeConfigFile PACKAGE_NAME)

    set(DEF_INSTALL_CMAKE_DIR lib/cmake/${PACKAGE_NAME})

	# Version file
	include(CMakePackageConfigHelpers)
	write_basic_package_version_file(
	  "${PROJECT_BINARY_DIR}/${PACKAGE_NAME}ConfigVersion.cmake"
	  VERSION ${${PACKAGE_NAME}_VERSION}
	  COMPATIBILITY SameMajorVersion
	)

	# Config file
	file(RELATIVE_PATH CONF_REL_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/${DEF_INSTALL_CMAKE_DIR}" "${CMAKE_INSTALL_PREFIX}/include")
	file(RELATIVE_PATH CONF_REL_LIB_DIR "${CMAKE_INSTALL_PREFIX}/${DEF_INSTALL_CMAKE_DIR}" "${CMAKE_INSTALL_PREFIX}/lib")
	configure_file(${CONFIG_TEMPLATE_PATH}/Config.cmake.in "${PROJECT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake" @ONLY)
	message(STATUS "Wrote ${PROJECT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake")

	# Install config, version and exports files (for find scripts)
	install(
		FILES
			"${PROJECT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake"
			"${PROJECT_BINARY_DIR}/${PACKAGE_NAME}ConfigVersion.cmake"
		DESTINATION
			"${CMAKE_INSTALL_PREFIX}/${DEF_INSTALL_CMAKE_DIR}"
	)
	install(EXPORT ${PACKAGE_NAME}-exports DESTINATION ${DEF_INSTALL_CMAKE_DIR})

endfunction()