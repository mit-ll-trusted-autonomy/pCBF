#
# DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.
#
# This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001.
# Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of
# the Under Secretary of Defense for Research and Engineering.
#
# © 2023 Massachusetts Institute of Technology.
#
# The software/firmware is provided to you on an As-Is basis
#
# Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014).
# Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above.
# Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.
#
#--------------------------------------------------------
# The CMakeLists.txt for:                       pCBFCore
# Author(s):                              James Usevitch
#--------------------------------------------------------

# Find Eigen Libraries
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

#[[This is a nasty hack--the linker couldn't find
   one of the functions in the libraries.]]
SET(ABSEIL_SRC
    external/abseil-cpp/absl/status/statusor.cc
)

SET(SRC
  CBFCore.cpp
  CBFCore_Info.cpp
  main.cpp
  ${ABSEIL_SRC}
)

# For OSQP-cpp
# TODO: Make this more general so this can be relocated.

ADD_SUBDIRECTORY(external/abseil-cpp)


ADD_EXECUTABLE(pCBFCore ${SRC})
TARGET_INCLUDE_DIRECTORIES(pCBFCore PUBLIC
    ./external/eigen-3.3.7
    ./external/osqp/include
    ./external/osqp-cpp/include
    ./external/abseil-cpp
)

set(OSQP_LIBRARIES
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/libosqp-cpp.a
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/libosqp.a
    absl::base
    absl::utility
    absl::meta
    absl::status
)

TARGET_LINK_LIBRARIES(pCBFCore
   ${MOOS_LIBRARIES}
   apputil
   mbutil
   m
   pthread
#    "$<LINK_GROUP:RESCAN,${OSQP_LIBRARIES}>"
    ${OSQP_LIBRARIES}
    dl
)

