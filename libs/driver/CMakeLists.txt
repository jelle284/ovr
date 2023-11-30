## DRIVER ##

file(GLOB DRIVER_GLOB src/*.h src/*.cpp)

add_library(driver_ovr SHARED
	${DRIVER_GLOB}
)

target_include_directories(driver_ovr PUBLIC
	src/*
	${OVR_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${CMAKE_SOURCE_DIR}/thirdparty/openvr/headers
)

target_link_libraries(driver_ovr PUBLIC
	optimized ${OVR_LIBS_RELEASE} debug ${OVR_LIBS_DEBUG}
	${OpenCV_LIBS}
)
	
set_target_properties(driver_ovr
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/driver/bin/win64
    LIBRARY_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/driver/bin/win64
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_SOURCE_DIR}/driver/bin/win64
)