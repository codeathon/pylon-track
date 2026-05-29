cmake_minimum_required(VERSION 3.16)
project(ferret_tracker)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

# All executables go to build/bin/ (e.g. build/bin/ferret_tracker).
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# ── Sources ───────────────────────────────────────────────────────────────────
set(SOURCES
	src/main.cpp
	src/camera_config.cpp
	src/tracker.cpp
	src/ferret_tracker.cpp
	src/display.cpp
)

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

find_package(OpenCV QUIET COMPONENTS core imgproc video highgui)

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
			find_package(OpenCV QUIET COMPONENTS core imgproc video highgui)
			if(OpenCV_FOUND)
				break()
			endif()
		endif()
	endforeach()
endif()

if(NOT OpenCV_FOUND)
	message(FATAL_ERROR
		"OpenCV not found (need components: core, imgproc, video, highgui).\n"
		"  Ubuntu/Debian:  sudo apt update && sudo apt install -y libopencv-dev build-essential cmake\n"
		"  Then reconfigure, or set OpenCV_DIR explicitly, e.g.:\n"
		"    cmake -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 ..")
endif()

message(STATUS "OpenCV version : ${OpenCV_VERSION}")
message(STATUS "OpenCV includes: ${OpenCV_INCLUDE_DIRS}")
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

	target_include_directories(ferret_tracker PRIVATE
		${CMAKE_SOURCE_DIR}/include
		${PYLON_ROOT}/include
		${OpenCV_INCLUDE_DIRS}
	)

	target_link_directories(ferret_tracker PRIVATE ${PYLON_ROOT}/lib)

	target_link_libraries(ferret_tracker
		${PYLON_LINK_FLAGS}
		${OpenCV_LIBS}
		pthread
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

	message(STATUS
		"Linux targets available:\n"
		"  make ferret_tracker   — build → bin/ferret_tracker\n"
		"  make run_rt           — run with SCHED_FIFO priority (low jitter)\n"
		"  make install_udev     — install udev rules for USB camera access")

else()
	message(FATAL_ERROR "Unsupported platform — Linux only")
endif()

# ── Build summary ─────────────────────────────────────────────────────────────
message(STATUS "Build type : ${CMAKE_BUILD_TYPE}")
message(STATUS "pylon root : ${PYLON_ROOT}")
message(STATUS "OpenCV ver : ${OpenCV_VERSION}")
