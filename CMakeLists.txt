cmake_minimum_required(VERSION 3.16)
project(ferret_tracker)

set(CMAKE_CXX_STANDARD 17)

# ── nlohmann/json (camera_config.json) ────────────────────────────────────────
find_package(nlohmann_json 3.11.2 QUIET)
if(NOT nlohmann_json_FOUND)
	include(FetchContent)
	FetchContent_Declare(
		nlohmann_json
		URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
	)
	FetchContent_MakeAvailable(nlohmann_json)
endif()

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

# All executables go to build/bin/ (e.g. build/bin/ferret_tracker).
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# ── Sources ───────────────────────────────────────────────────────────────────
set(ARENA_PIPELINE_SOURCES
	src/camera/camera_config.cpp
	src/camera/camera_calib.cpp
	src/tracker/tracker.cpp
	src/tracker/ferret_tracker.cpp
	src/tracker/display.cpp
	src/log/logger.cpp
	src/experiment/trial_state.cpp
	src/experiment/arena_config.cpp
	src/experiment/session_recorder.cpp
	src/experiment/op_timing.cpp
	src/vision/camera_frame.cpp
	src/vision/tracking_frame.cpp
	src/vision/arena_mask.cpp
	src/vision/object_associator.cpp
	src/vision/tracking_pipeline.cpp
	src/vision/camera_tracking_service.cpp
)

# Orchestrator needs motor drivers — only linked into arena_experiment.
set(EXPERIMENT_ORCHESTRATOR_SOURCES
	src/experiment/state_manager.cpp
)

set(SOURCES
	src/main.cpp
	${ARENA_PIPELINE_SOURCES}
)

set(MOTOR_SOURCES
	src/motor/odrive_can.cpp
	src/motor/prey_motor.cpp
	src/motor/motion_planner.cpp
	src/motor/labjack_hbridge.cpp
	src/motor/shuttle_motor.cpp
	src/motor/chase_policy.cpp
	src/motor/chase_controller.cpp
	src/motor/hunt_event.cpp
)

set(CALIBRATE_SOURCES
	src/calibrate/setup_util.cpp
	src/calibrate/setup_runner.cpp
	src/calibrate/arena_calibrator.cpp
	src/calibrate/camera_calibrator.cpp
	src/calibrate/odrive_calibrator.cpp
	src/calibrate/labjack_calibrator.cpp
	src/calibrate/charuco_board.cpp
)

# ── Optional LabJack LJM (shuttle motor H-bridge DIO) ────────────────────────
option(ENABLE_LABJACK "Link LabJack LJM for shuttle motor control" ON)
set(LJM_ROOT "/usr/local/lib" CACHE PATH "LabJack LJM library directory")

# ── Compiler flags ────────────────────────────────────────────────────────────
add_compile_options(-O3 -march=native -Wall -Wextra)

# ── pylon SDK ─────────────────────────────────────────────────────────────────
# Default install path is /opt/pylon on both Linux and macOS.
# Override at configure time: cmake -DPYLON_ROOT=/your/path ..
set(PYLON_ROOT "/opt/pylon" CACHE PATH "Path to pylon SDK installation")

if(NOT EXISTS "${PYLON_ROOT}/include/pylon/PylonIncludes.h")
	message(FATAL_ERROR
		"pylon SDK not found at '${PYLON_ROOT}'.\n"
		"Install from https://www.baslerweb.com/en/software/pylon/ "
		"or set -DPYLON_ROOT=<path>")
endif()

# ── OpenCV ────────────────────────────────────────────────────────────────────
# Install dev package first, e.g. Ubuntu: sudo apt install libopencv-dev
# If cmake still fails, pass: -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4
if(NOT OpenCV_DIR AND DEFINED ENV{OpenCV_DIR})
	set(OpenCV_DIR "$ENV{OpenCV_DIR}" CACHE PATH "OpenCV cmake config directory")
endif()

find_package(OpenCV QUIET COMPONENTS core imgproc video highgui calib3d imgcodecs)

if(NOT OpenCV_FOUND)
	# Probe common install locations before failing with a helpful message.
	foreach(_opencv_cmake_dir
		"/usr/lib/x86_64-linux-gnu/cmake/opencv4"
		"/usr/lib/cmake/opencv4"
		"/usr/local/lib/cmake/opencv4"
		"/opt/homebrew/lib/cmake/opencv4"
	)
		if(EXISTS "${_opencv_cmake_dir}/OpenCVConfig.cmake")
			set(OpenCV_DIR "${_opencv_cmake_dir}" CACHE PATH "OpenCV cmake config directory" FORCE)
			find_package(OpenCV QUIET COMPONENTS core imgproc video highgui calib3d imgcodecs)
			if(OpenCV_FOUND)
				break()
			endif()
		endif()
	endforeach()
endif()

if(NOT OpenCV_FOUND)
	message(FATAL_ERROR
		"OpenCV not found (need components: core, imgproc, video, highgui, calib3d, imgcodecs).\n"
		"  Ubuntu/Debian:  sudo apt update && sudo apt install -y libopencv-dev libopencv-contrib-dev\n"
		"  Then reconfigure, or set OpenCV_DIR explicitly, e.g.:\n"
		"    cmake -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 ..")
endif()

# ChArUco setup needs aruco symbols. On OpenCV ≥4.7/4.10 they live in
# opencv_objdetect (and may still ship a thin opencv_aruco); link both when present
# so ld does not fail with "DSO missing from command line".
find_path(PYLON_TRACK_ARUCO_INCLUDE opencv2/aruco.hpp
	HINTS ${OpenCV_INCLUDE_DIRS})
if(NOT PYLON_TRACK_ARUCO_INCLUDE)
	message(FATAL_ERROR
		"OpenCV aruco headers not found — install libopencv-contrib-dev (ChArUco camera setup)")
endif()
set(PYLON_TRACK_ARUCO_LIBS "")
foreach(_aruco_lib_name opencv_aruco opencv_objdetect)
	find_library(_aruco_lib_${_aruco_lib_name} NAMES ${_aruco_lib_name}
		HINTS ${OpenCV_LIB_DIR} /usr/lib/x86_64-linux-gnu /usr/local/lib /opt/homebrew/lib)
	if(_aruco_lib_${_aruco_lib_name})
		list(APPEND PYLON_TRACK_ARUCO_LIBS ${_aruco_lib_${_aruco_lib_name}})
		message(STATUS "OpenCV ${_aruco_lib_name}: ${_aruco_lib_${_aruco_lib_name}}")
	endif()
endforeach()
if(NOT PYLON_TRACK_ARUCO_LIBS)
	message(WARNING
		"opencv_aruco / opencv_objdetect not found — arena_experiment setup link may fail")
endif()

message(STATUS "OpenCV version : ${OpenCV_VERSION}")
message(STATUS "OpenCV includes: ${OpenCV_INCLUDE_DIRS}")

# ── cnpy (read/write calib.npz from setup; vendored — no FetchContent/git) ──
find_package(ZLIB REQUIRED)
add_library(cnpy STATIC third_party/cnpy/cnpy.cpp)
target_include_directories(cnpy PUBLIC third_party/cnpy)
target_link_libraries(cnpy PUBLIC ZLIB::ZLIB)

message(STATUS "Binary output  : ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

# ── Platform-specific configuration ──────────────────────────────────────────
if(UNIX)
	# ── Linux ──────────────────────────────────────────────────────────────
	message(STATUS "Configuring for Linux")

	# pylon uses a helper script (pylon-config) on Linux to expose the exact
	# library list for the installed version. Use it when available; fall back
	# to explicit names otherwise.
	find_program(PYLON_CONFIG pylon-config HINTS "${PYLON_ROOT}/bin")

	if(PYLON_CONFIG)
		execute_process(
			COMMAND ${PYLON_CONFIG} --libs
			OUTPUT_VARIABLE PYLON_LIBS_RAW OUTPUT_STRIP_TRAILING_WHITESPACE)
		execute_process(
			COMMAND ${PYLON_CONFIG} --cflags-only-I
			OUTPUT_VARIABLE PYLON_CFLAGS_RAW OUTPUT_STRIP_TRAILING_WHITESPACE)

		# Convert "-lfoo -lbar" → cmake list of library names
		separate_arguments(PYLON_LINK_FLAGS UNIX_COMMAND "${PYLON_LIBS_RAW}")
		separate_arguments(PYLON_INC_FLAGS  UNIX_COMMAND "${PYLON_CFLAGS_RAW}")

		message(STATUS "pylon-config libs  : ${PYLON_LIBS_RAW}")
		message(STATUS "pylon-config cflags: ${PYLON_CFLAGS_RAW}")
	else()
		# pylon-config not on PATH — use known library names for pylon 6/7
		message(STATUS "pylon-config not found, using default library names")
		set(PYLON_LINK_FLAGS
			-lpylonbase
			-lpylonutility
			-lGCBase_gcc_v3_1_Basler_pylon
			-lGenApi_gcc_v3_1_Basler_pylon
		)
		set(PYLON_INC_FLAGS "-I${PYLON_ROOT}/include")
	endif()

	add_executable(ferret_tracker ${SOURCES})

	add_executable(arena_experiment
		src/arena_experiment_main.cpp
		${ARENA_PIPELINE_SOURCES}
		${EXPERIMENT_ORCHESTRATOR_SOURCES}
		${CALIBRATE_SOURCES}
	)

	add_library(arena_motor STATIC ${MOTOR_SOURCES})
	# chase_policy / motor_config pull arena_config → arena_mask (OpenCV types).
	target_include_directories(arena_motor PUBLIC
		${CMAKE_SOURCE_DIR}/include
		${OpenCV_INCLUDE_DIRS}
	)
	target_link_libraries(arena_motor PUBLIC pthread ${OpenCV_LIBS})

	if(ENABLE_LABJACK)
		find_path(LJM_INCLUDE_DIR LabJackM.h
			HINTS /usr/include /usr/local/include ${LJM_ROOT}/../include)
		find_library(LJM_LIBRARY NAMES labjacklm
			HINTS ${LJM_ROOT} /usr/lib /usr/local/lib)
		if(LJM_INCLUDE_DIR AND LJM_LIBRARY)
			# PUBLIC so arena_experiment picks up LJM when linking the static lib.
			target_compile_definitions(arena_motor PUBLIC PYLON_TRACK_HAS_LJM=1)
			target_include_directories(arena_motor PUBLIC ${LJM_INCLUDE_DIR})
			target_link_libraries(arena_motor PUBLIC ${LJM_LIBRARY})
			message(STATUS "LabJack LJM   : ${LJM_LIBRARY}")
		else()
			message(STATUS "LabJack LJM   : not found — shuttle motor LabJack backend stubbed")
		endif()
	endif()

	target_include_directories(ferret_tracker PRIVATE
		${CMAKE_SOURCE_DIR}/include
		${PYLON_ROOT}/include
		${OpenCV_INCLUDE_DIRS}
	)

	target_link_directories(ferret_tracker PRIVATE ${PYLON_ROOT}/lib)

	target_link_libraries(ferret_tracker
		${PYLON_LINK_FLAGS}
		${OpenCV_LIBS}
		nlohmann_json::nlohmann_json
		cnpy
		ZLIB::ZLIB
		pthread
	)

	target_include_directories(arena_experiment PRIVATE
		${CMAKE_SOURCE_DIR}/include
		${PYLON_ROOT}/include
		${OpenCV_INCLUDE_DIRS}
	)
	target_link_directories(arena_experiment PRIVATE ${PYLON_ROOT}/lib)
	target_link_libraries(arena_experiment
		${PYLON_LINK_FLAGS}
		${OpenCV_LIBS}
		${PYLON_TRACK_ARUCO_LIBS}
		nlohmann_json::nlohmann_json
		cnpy
		ZLIB::ZLIB
		arena_motor
		pthread
	)
	set_target_properties(arena_experiment PROPERTIES
		INSTALL_RPATH "${PYLON_ROOT}/lib"
		BUILD_RPATH   "${PYLON_ROOT}/lib"
	)

	# Copy camera config beside the executable for default runtime lookup.
	configure_file(
		${CMAKE_SOURCE_DIR}/src/camera/camera_config.json
		${CMAKE_BINARY_DIR}/bin/camera_config.json
		COPYONLY
	)
	add_custom_target(copy_camera_config ALL
		DEPENDS ${CMAKE_BINARY_DIR}/bin/camera_config.json
	)
	add_dependencies(ferret_tracker copy_camera_config)

	configure_file(
		${CMAKE_SOURCE_DIR}/config/arena_experiment.json
		${CMAKE_BINARY_DIR}/bin/config/arena_experiment.json
		COPYONLY
	)
	add_custom_target(copy_arena_config ALL
		DEPENDS ${CMAKE_BINARY_DIR}/bin/config/arena_experiment.json
	)
	add_dependencies(ferret_tracker copy_arena_config)
	add_dependencies(arena_experiment copy_camera_config copy_arena_config)

	# Lab motor move smoke test (MotionPlanner distance/speed) — no camera/pylon.
	add_executable(test_odrive_move
		tests/odrive_move_test.cpp
		src/experiment/arena_config.cpp
		src/log/logger.cpp
		src/calibrate/setup_util.cpp
	)
	target_include_directories(test_odrive_move PRIVATE
		${CMAKE_SOURCE_DIR}/include
		${OpenCV_INCLUDE_DIRS}
	)
	target_link_libraries(test_odrive_move
		arena_motor
		nlohmann_json::nlohmann_json
		${OpenCV_LIBS}
		pthread
	)
	add_dependencies(test_odrive_move copy_arena_config)

	# Offline MotionPlanner unit tests (fake IMotor — no CAN hardware).
	add_executable(test_motion_planner
		tests/motion_planner_test.cpp
		src/log/logger.cpp
		src/motor/motion_planner.cpp
	)
	target_include_directories(test_motion_planner PRIVATE
		${CMAKE_SOURCE_DIR}/include
	)
	target_link_libraries(test_motion_planner pthread)

	# Offline hunt-event edge / re-arm unit tests (no camera/CAN).
	add_executable(test_hunt_event
		tests/hunt_event_test.cpp
		src/motor/hunt_event.cpp
	)
	target_include_directories(test_hunt_event PRIVATE
		${CMAKE_SOURCE_DIR}/include
	)

	# RPATH: embed pylon lib path so the binary runs without LD_LIBRARY_PATH
	set_target_properties(ferret_tracker PROPERTIES
		INSTALL_RPATH "${PYLON_ROOT}/lib"
		BUILD_RPATH   "${PYLON_ROOT}/lib"
	)

	# ── Real-time scheduling target (Linux only) ──────────────────────────
	# Wraps the binary with chrt -f 50 for SCHED_FIFO priority.
	# Requires: sudo setcap cap_sys_nice+ep bin/ferret_tracker  (from build dir)
	add_custom_target(run_rt
		COMMAND chrt -f 50 $<TARGET_FILE:ferret_tracker>
		DEPENDS ferret_tracker
		COMMENT "Launching ferret_tracker with real-time SCHED_FIFO priority 50"
	)

	# ── USB udev rules install target ─────────────────────────────────────
	# Basler ships udev rules with pylon. This target installs them so the
	# camera is accessible without root.
	set(UDEV_RULES_SRC "${PYLON_ROOT}/share/pylon/udev/69-basler-cameras.rules")
	set(UDEV_RULES_DST "/etc/udev/rules.d/69-basler-cameras.rules")

	add_custom_target(install_udev
		COMMAND sudo cp ${UDEV_RULES_SRC} ${UDEV_RULES_DST}
		COMMAND sudo udevadm control --reload-rules
		COMMAND sudo udevadm trigger
		COMMENT "Installing Basler udev rules → ${UDEV_RULES_DST}"
	)

	# ── SocketCAN can0 host bring-up (ODrive USB-CAN adapter) ─────────────
	# Lab flow: cmake && make — ALL (and arena_experiment) run this with sudo.
	# Success criterion: can0 administratively UP, or make fails.
	include(GNUInstallDirs)
	option(INSTALL_CAN0_HOST "Bring up SocketCAN can0 during make (needs sudo)" ON)
	if(INSTALL_CAN0_HOST)
		configure_file(
			${CMAKE_SOURCE_DIR}/scripts/pylon-track-can0.service.in
			${CMAKE_BINARY_DIR}/pylon-track-can0.service
			@ONLY
		)
		add_custom_target(can0_host ALL
			COMMAND sudo ${CMAKE_SOURCE_DIR}/scripts/install-can0-host.sh
				${CMAKE_BINARY_DIR}/pylon-track-can0.service
				${CMAKE_SOURCE_DIR}/scripts/can0-up.sh
				${CMAKE_INSTALL_FULL_SBINDIR}
			DEPENDS ${CMAKE_BINARY_DIR}/pylon-track-can0.service
			COMMENT "SocketCAN: install drivers/unit and bring can0 UP (sudo)"
			VERBATIM
			USES_TERMINAL
		)
		# So `make arena_experiment` alone still brings can0 UP.
		add_dependencies(arena_experiment can0_host)

		install(PROGRAMS ${CMAKE_SOURCE_DIR}/scripts/can0-up.sh
			DESTINATION ${CMAKE_INSTALL_SBINDIR})
		install(PROGRAMS ${CMAKE_SOURCE_DIR}/scripts/install-can0-host.sh
			DESTINATION ${CMAKE_INSTALL_SBINDIR})
		install(FILES ${CMAKE_BINARY_DIR}/pylon-track-can0.service
			DESTINATION lib/systemd/system)
		install(FILES ${CMAKE_SOURCE_DIR}/scripts/80-can0.network
			DESTINATION ${CMAKE_INSTALL_DATADIR}/pylon-track/systemd)
	endif()

	message(STATUS
		"Linux targets available:\n"
		"  make                  — build + bring can0 UP (sudo; INSTALL_CAN0_HOST)\n"
		"  make arena_experiment — build experiment + bring can0 UP\n"
		"  make test_odrive_move — chain distance/speed move via MotionPlanner\n"
		"  make test_motion_planner — offline MotionPlanner unit tests (no CAN)\n"
		"  make test_hunt_event — offline hunt-event edge/re-arm unit tests\n"
		"  make ferret_tracker   — build → bin/ferret_tracker\n"
		"  make run_rt           — run with SCHED_FIFO priority (low jitter)\n"
		"  make install_udev     — install udev rules for USB camera access\n"
		"  -DINSTALL_CAN0_HOST=OFF to skip SocketCAN on make")

	# ── Calibration test suite (tests/) ───────────────────────────────────
	# Camera-attached tools: parameter sweep, latency benchmark, mount
	# height validation. Reuses the PYLON_*/OpenCV variables set above.
	option(BUILD_CALIBRATION_TESTS "Build calibration test suite" ON)
	if(BUILD_CALIBRATION_TESTS)
		add_subdirectory(tests)
	endif()

else()
	message(FATAL_ERROR "Unsupported platform — Linux only")
endif()

# ── Build summary ─────────────────────────────────────────────────────────────
message(STATUS "Build type : ${CMAKE_BUILD_TYPE}")
message(STATUS "pylon root : ${PYLON_ROOT}")
message(STATUS "OpenCV ver : ${OpenCV_VERSION}")
