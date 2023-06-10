## FIND OVR ##

find_path(OVR_INCLUDE_DIRS
		NAMES ovr.h
		HINTS "libs/ovr"
		PATH_SUFFIXES "include")

find_library(OVR_LIBS_RELEASE
  NAMES ovr_lib
  PATH_SUFFIXES "bin")

 find_library(OVR_LIBS_DEBUG
  NAMES ovr_lib
  PATH_SUFFIXES "debug")

message("Found OVR at (${OVR_INCLUDE_DIRS}), (${OVR_LIBS_RELEASE}), (${OVR_LIBS_DEBUG})")