# 
#  Copyright 2009 AIST-CNRS
#
#  Author: Florent Lamiraux, Nicolas Perrin
# 

# Configure package.dox file
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/package.dox.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/package.dox
)


ADD_CUSTOM_COMMAND(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/latex/doc.tex
  COMMAND ${DOXYGEN_EXECUTABLE} \"${CMAKE_CURRENT_BINARY_DIR}/package.dox\"
)

ADD_CUSTOM_TARGET(documentation ALL
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/latex/doc.tex)

