## OVR LIB ""

file(GLOB CAMERA_GLOB src/camera/*.h src/camera/*.cpp)
file(GLOB DEVICE_GLOB src/devices/*.h src/devices/*.cpp)
file(GLOB COMMON_GLOB src/common/*.h src/common/*.cpp)

add_library(
	ovr_lib SHARED
		include/ovr.h
		src/ovr.cpp
		${PS3EYE_SOURCE}
		${CAMERA_GLOB}
		${DEVICE_GLOB}
		${COMMON_GLOB}
)

target_include_directories(
	ovr_lib PUBLIC 
		src/camera
		src/devices
		src/common
		include
		${LibUSB_INCLUDE_DIRS}
		${PS3EYE_INCLUDE_DIRS}
		${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
	ovr_lib PUBLIC
		${SOCKET_LIBS}
		${LibUSB_LIBS}
		${OpenCV_LIBS}
)
