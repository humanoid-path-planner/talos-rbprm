cmake_minimum_required(VERSION 3.1)

set(PROJECT_NAME talos-rbprm)
set(PROJECT_DESCRIPTION "Database for talos robot using hpp-rbprm")

set(CUSTOM_HEADER_DIR "hpp/talos-rbprm")
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CXX_DISABLE_WERROR true)

include(cmake/hpp.cmake)
include(cmake/python.cmake)

compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

add_project_dependency(example-robot-data REQUIRED)

findpython()

add_library(${PROJECT_NAME} INTERFACE)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

set(CATKIN_PACKAGE_SHARE_DESTINATION
    ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

install(DIRECTORY data/talos/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

string(REGEX REPLACE "-" "_" PY_NAME ${PROJECT_NAME})

set(${PROJECT_NAME}_PYTHON_FILES __init__.py talos_abstract.py
                                 talos_fixedUpper.py talos.py)

foreach(SOURCE ${${PROJECT_NAME}_PYTHON_FILES})
  python_install_on_site(${PY_NAME} ${SOURCE})
endforeach(SOURCE ${${PROJECT_NAME}_PYTHON_SOURCES})

install(FILES package.xml DESTINATION share/${PROJECT_NAME})
