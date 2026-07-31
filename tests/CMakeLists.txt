# Calibration test suite — camera-attached executables.
# Inherits PYLON_* and OpenCV variables from the root CMakeLists (same SDK
# discovery; do not duplicate it here).

# Production sources reused by the suites (tracker pipeline + config loader).
# main.cpp/display.cpp excluded: the suites have their own entry points and
# never open a display window.
set(CALIB_PROD_SOURCES
	${CMAKE_SOURCE_DIR}/src/camera/camera_config.cpp
	${CMAKE_SOURCE_DIR}/src/camera/camera_calib.cpp
	${CMAKE_SOURCE_DIR}/src/tracker/tracker.cpp
	${CMAKE_SOURCE_DIR}/src/tracker/ferret_tracker.cpp
	${CMAKE_SOURCE_DIR}/src/log/logger.cpp
	${CMAKE_SOURCE_DIR}/src/experiment/trial_state.cpp
	${CMAKE_SOURCE_DIR}/src/experiment/session_recorder.cpp
	${CMAKE_SOURCE_DIR}/src/vision/camera_frame.cpp
	${CMAKE_SOURCE_DIR}/src/vision/tracking_frame.cpp
	${CMAKE_SOURCE_DIR}/src/vision/arena_mask.cpp
	${CMAKE_SOURCE_DIR}/src/vision/object_associator.cpp
	${CMAKE_SOURCE_DIR}/src/vision/tracking_pipeline.cpp
	${CMAKE_SOURCE_DIR}/src/vision/camera_tracking_service.cpp
)

set(CALIB_COMMON_SOURCES
	common/test_session.cpp
	common/image_metrics.cpp
	common/measurement_run.cpp
	common/camera_sweep.cpp
	common/telemetry_run.cpp
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
	cnpy
	ZLIB::ZLIB
	pthread
)

# One executable per suite -> build/bin/.
# param_sweep handles both single-parameter and resolution-preset specs.
foreach(suite param_sweep latency_suite mount_height_suite one_time_suite telemetry_suite)
	if(suite STREQUAL "param_sweep")
		set(exe_name test_param_sweep)
	elseif(suite STREQUAL "latency_suite")
		set(exe_name test_latency)
	elseif(suite STREQUAL "one_time_suite")
		set(exe_name test_one_time_setup)
	elseif(suite STREQUAL "telemetry_suite")
		set(exe_name test_telemetry)
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

# Motor-only distance/duration move test — no camera or Pylon linkage needed,
# unlike the camera-attached suites above. Links the arena_motor static lib
# (root CMakeLists.txt) rather than recompiling the motor sources a second
# time, same pattern as test_odrive_move.
add_executable(test_distance_moving
	distance_moving_suite.cpp
	${CMAKE_SOURCE_DIR}/src/experiment/arena_config.cpp
	${CMAKE_SOURCE_DIR}/src/calibrate/setup_util.cpp
	${CMAKE_SOURCE_DIR}/src/log/logger.cpp
)
target_include_directories(test_distance_moving PRIVATE
	${CMAKE_SOURCE_DIR}/include
	${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(test_distance_moving PRIVATE
	arena_motor
	${OpenCV_LIBS}
	nlohmann_json::nlohmann_json
	pthread
)
