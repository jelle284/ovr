## FIND LIB USB ##

find_path(LibUSB_INCLUDE_DIRS
		NAMES libusb.h
		PATH_SUFFIXES "include" "libusb" "libusb-1.0")

find_library(LibUSB_LIBS
  NAMES libusb-1.0
  PATH_SUFFIXES "MS64" "dll")

message("Found LibUSB at (${LibUSB_INCLUDE_DIRS}), (${LibUSB_LIBS})")