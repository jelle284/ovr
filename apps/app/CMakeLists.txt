## APP ##

FILE(GLOB APP_FILES src/*.h src/*.cpp)
FILE(GLOB UI_SOURCE_FILES src/ui/*.h src/ui/*.cpp)

add_executable(ovr_app WIN32
	${APP_FILES}
	${UI_SOURCE_FILES}
)
	
target_include_directories(ovr_app INTERFACE
	src
	PUBLIC
	${OVR_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
    ${LibUSB_INCLUDE_DIRS}
)

target_link_libraries(ovr_app PUBLIC
	optimized ${OVR_LIBS_RELEASE} debug ${OVR_LIBS_DEBUG}
	${OpenCV_LIBS}
	${LibUSB_LIBS}
)