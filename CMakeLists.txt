cmake_minimum_required(VERSION 3.10)

set(PROJECT_NAME talos-rbprm)
set(PROJECT_DESCRIPTION "Database for talos robot using hpp-rbprm")

set(CUSTOM_HEADER_DIR "hpp/talos-rbprm")
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CXX_DISABLE_WERROR true)

# Check if the submodule cmake have been initialized
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(EXISTS "${JRL_CMAKE_MODULES}/base.cmake")
  message(STATUS "JRL cmakemodules found in 'cmake/' git submodule")
else()
  find_package(jrl-cmakemodules QUIET CONFIG)
  if(jrl-cmakemodules_FOUND)
    get_property(
      JRL_CMAKE_MODULES
      TARGET jrl-cmakemodules::jrl-cmakemodules
      PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
    message(STATUS "JRL cmakemodules found on system at ${JRL_CMAKE_MODULES}")
  elseif(${CMAKE_VERSION} VERSION_LESS "3.14.0")
    message(
      FATAL_ERROR
        "\nCan't find jrl-cmakemodules. Please either:\n"
        "  - use git submodule: 'git submodule update --init'\n"
        "  - or install https://github.com/jrl-umi3218/jrl-cmakemodules\n"
        "  - or upgrade your CMake version to >= 3.14 to allow automatic fetching\n"
    )
  else()
    message(STATUS "JRL cmakemodules not found. Let's fetch it.")
    include(FetchContent)
    FetchContent_Declare(
      "jrl-cmakemodules"
      GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git")
    FetchContent_MakeAvailable("jrl-cmakemodules")
    FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
  endif()
endif()

include("${JRL_CMAKE_MODULES}/hpp.cmake")
include("${JRL_CMAKE_MODULES}/python.cmake")

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
