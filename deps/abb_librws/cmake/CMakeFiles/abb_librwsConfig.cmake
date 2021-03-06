# - Config file for the abb_librws package
# It defines the following variable
#  abb_librws_LIBRARIES - libraries to link against

include(CMakeFindDependencyMacro)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Find dependencies
find_dependency(Poco 1.4.3 REQUIRED COMPONENTS Foundation Net Util XML)

# Our library dependencies (contains definitions for IMPORTED targets)
include("${CMAKE_CURRENT_LIST_DIR}/abb_librwsTargets.cmake")

# These are IMPORTED targets created by abb_librwsTargets.cmake
set(abb_librws_LIBRARIES abb_librws::abb_librws)
