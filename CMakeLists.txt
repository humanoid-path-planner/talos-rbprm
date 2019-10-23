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

PROJECT(${PROJECT_NAME} CXX)

# search for python
FINDPYTHON()

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

SET(CATKIN_PACKAGE_SHARE_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})


install(DIRECTORY
  data/talos/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install (FILES
  src/hpp/corbaserver/rbprm/talos_abstract/robot.py
  src/hpp/corbaserver/rbprm/talos_abstract/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/rbprm/talos_abstract)

install (FILES
  src/hpp/corbaserver/rbprm/talos_fixedUpper/robot.py
  src/hpp/corbaserver/rbprm/talos_fixedUpper/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/rbprm/talos_fixedUpper)

install (FILES
  src/hpp/corbaserver/rbprm/talos/robot.py
  src/hpp/corbaserver/rbprm/talos/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/rbprm/talos)

SETUP_PROJECT_FINALIZE()
