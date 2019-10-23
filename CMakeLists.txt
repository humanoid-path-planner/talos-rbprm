# Copyright (c) 2012 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-rbprm-corba.
# hpp-rbprm-corba is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-rbprm-corba is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-rbprm-corba.  If not, see
# <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(PROJECT_NAME talos-rbprm)
SET(PROJECT_DESCRIPTION "Database for talos robot using hpp-rbprm")

INCLUDE(cmake/hpp.cmake)
INCLUDE(cmake/python.cmake)

SET(CXX_DISABLE_WERROR true)

SET(CUSTOM_HEADER_DIR "hpp/talos-rbprm")

CMAKE_POLICY(SET CMP0048 OLD)  # we compute the PROJECT_VERSION manually
PROJECT(${PROJECT_NAME} CXX)

FINDPYTHON()

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

SET(CATKIN_PACKAGE_SHARE_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

INSTALL(DIRECTORY
  data/talos/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

STRING(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})

SET(${PROJECT_NAME}_PYTHON_FILES
  __init__.py
  talos_abstract.py
  talos_fixedUpper.py
  talos.py
  )

FOREACH(SOURCE ${${PROJECT_NAME}_PYTHON_FILES})
  PYTHON_INSTALL_ON_SITE(${PY_NAME} ${SOURCE})
ENDFOREACH(SOURCE ${${PROJECT_NAME}_PYTHON_SOURCES})
