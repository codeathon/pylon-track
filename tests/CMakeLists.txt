# Calibration test suite — camera-attached executables.
# Inherits PYLON_* and OpenCV variables from the root CMakeLists (same SDK
# discovery; do not duplicate it here).

# Production sources reused by the suites (tracker pipeline + config loader).
# main.cpp/display.cpp excluded: the suites have their own entry points and
# never open a display window.
set(CALIB_PROD_SOURCES
	${CMAKE_SOURCE_DIR}/src/camera_config.cpp
	${CMAKE_SOURCE_DIR}/src/tracker.cpp
	${CMAKE_SOURCE_DIR}/src/ferret_tracker.cpp
	${CMAKE_SOURCE_DIR}/src/logger.cpp
)

set(CALIB_COMMON_SOURCES
	common/test_session.cpp
	common/image_metrics.cpp
	common/measurement_run.cpp
	common/camera_sweep.cpp
)

# Shared static lib so the production .cpp files compile once, not 3x.
add_library(calib_common STATIC ${CALIB_PROD_SOURCES} ${CALIB_COMMON_SOURCES})

target_include_directories(calib_common PUBLIC
	${CMAKE_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/common
	${PYLON_ROOT}/include
	${OpenCV_INCLUDE_DIRS}
)

target_link_directories(calib_common PUBLIC ${PYLON_ROOT}/lib)

target_link_libraries(calib_common PUBLIC
	${PYLON_LINK_FLAGS}
	${OpenCV_LIBS}
	nlohmann_json::nlohmann_json
	pthread
)

# One executable per suite -> build/bin/.
# param_sweep handles both single-parameter and resolution-preset specs.
foreach(suite param_sweep latency_suite mount_height_suite one_time_suite)
	if(suite STREQUAL "param_sweep")
		set(exe_name test_param_sweep)
	elseif(suite STREQUAL "latency_suite")
		set(exe_name test_latency)
	elseif(suite STREQUAL "one_time_suite")
		set(exe_name test_one_time_setup)
	else()
		set(exe_name test_mount_height)
	endif()

	add_executable(${exe_name} ${suite}.cpp)
	target_link_libraries(${exe_name} calib_common)
	# Same RPATH treatment as ferret_tracker: run without LD_LIBRARY_PATH.
	set_target_properties(${exe_name} PROPERTIES
		INSTALL_RPATH "${PYLON_ROOT}/lib"
		BUILD_RPATH   "${PYLON_ROOT}/lib"
	)
endforeach()
