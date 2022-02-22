## FIND OVR ##

find_path(OVR_INCLUDE_DIRS
		NAMES ovr.h
		HINTS "libs/ovr"
		PATH_SUFFIXES "include")

find_library(OVR_LIBS
  NAMES ovr_lib
  PATH_SUFFIXES "bin")

message("Found OVR at (${OVR_INCLUDE_DIRS}), (${OVR_LIBS})")