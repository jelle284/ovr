## 				OVR PROJECT 				##

cmake_minimum_required(VERSION "3.18.0")

project("ovr")

set(CMAKE_CXX_STANDARD 14)

option(BUILD_TESTS "Build tests" OFF)

# PATHS
list(APPEND CMAKE_MODULE_PATH
 "${PROJECT_SOURCE_DIR}/cmake"
 )


list(APPEND CMAKE_PREFIX_PATH
 "D:/OpenCV/build"
 "D:/LibUSB"
 "${CMAKE_SOURCE_DIR}"
 )

find_path(STEAM_PATH
	NAMES "steam.exe"
	HINTS "D:/" "C:/"
	PATH_SUFFIXES "Steam"
)
message("Found steam in ${STEAM_PATH}")

# LIBRARIES
find_package(OpenCV CONFIG REQUIRED)

find_package(LibUSB MODULE REQUIRED)

find_package(OVR MODULE REQUIRED)

set(SOCKET_LIBS wsock32 ws2_32)

# PS3EYE Driver
find_path(PS3EYE_INCLUDE_DIRS
		NAMES "ps3eye.h"
		HINTS "thirdparty/PS3EYEDriver/src"
)
		
find_file(PS3EYE_SOURCE
	NAMES "ps3eye.cpp"
	HINTS ${PS3EYE_INCLUDE_DIRS}
)

message("Found PS3EYEDriver at (${PS3EYE_INCLUDE_DIRS}), (${PS3EYE_SOURCE})")

# OVR LIB
add_subdirectory(libs/ovr)

# APP
add_subdirectory(apps/app)

# DRIVER 
add_subdirectory(libs/driver)


# set outputs for targets
set_target_properties(
	ovr_lib
    PROPERTIES
	ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin
	LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/debug
    LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/debug
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/debug
)

set_target_properties(
	ovr_app
	PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY_RELEASE ${STEAM_PATH}/config/ovr
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/debug
)


# SANDBOX
FILE(GLOB UI_SOURCE_FILES apps/app/src/ui/*.h apps/app/src/ui/*.cpp)
add_executable(sandbox 
	apps/sandbox.cpp
	${UI_SOURCE_FILES}
)

message(UI_SOURCE_FILES="${UI_SOURCE_FILES}")

target_include_directories(sandbox PUBLIC
	apps/app/src/ui
	${OVR_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
    ${LibUSB_INCLUDE_DIRS}
)

target_link_libraries(sandbox PUBLIC
	optimized ${OVR_LIBS_RELEASE} debug ${OVR_LIBS_DEBUG}
	${OpenCV_LIBS}
	${LibUSB_LIBS}
)

set_target_properties(
	ovr_app
	PROPERTIES
	RUNTIME_OUTPUT_DIRECTORY_RELEASE ${STEAM_PATH}/config/ovr
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/debug
)

if(BUILD_TESTS)
# MONITOR TEST
add_executable(testmonitor 
	apps/tests/testmonitor.cpp
)

# HMD TEST
add_executable(testhmd 
	src/tests/testhmd.cpp
	${COMMON_GLOB}
	${DEVICES_GLOB}
	${CAMERA_GLOB}
	${PS3EYE})
	
target_include_directories(testhmd PRIVATE
	src/common
	src/tests
	src/devices
	src/camera
	${CAMERA_INCLUDE_DIR})
	
target_link_directories(testhmd PUBLIC
	${CAMERA_LINK_DIR})
	
target_link_libraries(testhmd PUBLIC
	${SOCKET_LIB}
	${CAMERA_LINK_LIB})
	
# CONTROLLER TEST
add_executable(testcontroller 
	src/tests/testcontroller.cpp
	${DEVICES_GLOB})

target_include_directories(testcontroller PRIVATE
	src/common
	src/tests
	src/devices)
	
target_link_libraries(testcontroller PUBLIC
	${SOCKET_LIB})
	
# CAMERA TEST
add_executable(testcamera 
	src/tests/testcamera.cpp
	${CAMERA_GLOB}
	${PS3EYE})
	
target_include_directories(testcamera PRIVATE
	src/camera
	src/common
	${CAMERA_INCLUDE_DIR}
	src/tests)
	
target_link_directories(testcamera PUBLIC
	${CAMERA_LINK_DIR})
	
target_link_libraries(testcamera optimized
	${CAMERA_LINK_LIB})
	
target_link_libraries(testcamera debug
	libusb-1.0.lib
	opencv_world440d.lib)
	
# LIBUSB TEST	
add_executable(testlibusb 
	src/tests/testlibusb.c)

target_include_directories(testlibusb PUBLIC
	$ENV{LIBUSB_DIR}/include/libusb-1.0)
	
target_link_directories(testlibusb PUBLIC
	$ENV{LIBUSB_DIR}/MS64/dll)
	
target_link_libraries(testlibusb 
	libusb-1.0.lib)
	

	
endif()
unset(BUILD_TESTS CACHE)