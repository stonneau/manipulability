# Base Io build system
# Written by there.exists.teslos<there.exists.teslos.gmail.com>
#
# Find drawstuff
FIND_PATH(DRAWSTUFF_INCLUDE_DIR drawstuff/drawstuff.h
    /usr/include
    /usr/local/include
)

SET(DRAW_NAMES ${DRAW_NAMES} drawstuff libdrawstuff)
FIND_LIBRARY(DRAWSTUFF_LIBRARY NAMES ${DRAW_NAMES} PATH)

IF(DRAWSTUFF_LIBRARY)
    MESSAGE(STATUS "Found DrawStuff library: ${DRAWSTUFF_LIBRARY}")
ELSE(DRAWSTUFF_LIBRARY)
    MESSAGE(STATUS "Coulddn't find DrawStuff library: ${DRAWSTUFF_LIBRARY}")
ENDIF(DRAWSTUFF_LIBRARY)

IF(DRAWSTUFF_INCLUDE_DIR AND DRAWSTUFF_LIBRARY)
SET(DRAWSTUFF_FOUND TRUE CACHE STRING "Whether DRAWSTUFF was found or not")
ENDIF(DRAWSTUFF_INCLUDE_DIR AND DRAWSTUFF_LIBRARY)

IF(DRAWSTUFF_FOUND)
    SET(CMAKE_C_FLAGS "-DdSINGLE")
IF(NOT DRAWSTUFF_FIND_QUIETLY)
MESSAGE(STATUS "Found DRAWSTUFF: ${DRAWSTUFF_LIBRARY}")
ENDIF (NOT DRAWSTUFF_FIND_QUIETLY)
ELSE(DRAWSTUFF_FOUND)
IF(DRAWSTUFF_FIND_REQUIRED)
MESSAGE(FATAL_ERROR "Could not find DRAWSTUFF")
ENDIF(DRAWSTUFF_FIND_REQUIRED)
ENDIF(DRAWSTUFF_FOUND)
