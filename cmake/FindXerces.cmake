
#
# Find the native Xerces includes and library
#
# XERCES_INCLUDE_DIR - where to find dom/dom.hpp, etc.
# XERCES_LIBRARIES   - List of fully qualified libraries to link against
# XERCES_FOUND       - Do not attempt to use Xerces if "no" or undefined.

FIND_PATH(XERCES_INCLUDE_DIR dom/dom.hpp
  /usr/local/include
  /usr/include
)

# There may be some API changes between Xerces 1.x and 2.x
# I'm not sure how to deal with that in a .cmake file
# Perhaps it should be up to the application to figure out the version and
# API specifics from macros set in the headers?

FIND_LIBRARY(XERCES_LIBRARY
  NAMES
    xerces-c_2
    xerces-c_2D
    xerces-c_1
    xerces-c_1D
  PATHS
    /usr/local/lib
    /usr/lib
)

IF(XERCES_INCLUDE_DIR)
  IF(XERCES_LIBRARY)
    SET( XERCES_LIBRARIES ${XERCES_LIBRARY} )
    SET( XERCES_FOUND "YES" )
  ENDIF(XERCES_LIBRARY)
ENDIF(XERCES_INCLUDE_DIR)