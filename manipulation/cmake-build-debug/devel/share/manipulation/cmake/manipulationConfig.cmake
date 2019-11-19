# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(manipulation_CONFIG_INCLUDED)
  return()
endif()
set(manipulation_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(manipulation_SOURCE_PREFIX /home/lorenzo/UWSim/src/Perception_and_Manipulation/manipulation)
  set(manipulation_DEVEL_PREFIX /home/lorenzo/UWSim/src/Perception_and_Manipulation/manipulation/cmake-build-debug/devel)
  set(manipulation_INSTALL_PREFIX "")
  set(manipulation_PREFIX ${manipulation_DEVEL_PREFIX})
else()
  set(manipulation_SOURCE_PREFIX "")
  set(manipulation_DEVEL_PREFIX "")
  set(manipulation_INSTALL_PREFIX /usr/local)
  set(manipulation_PREFIX ${manipulation_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'manipulation' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(manipulation_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/home/lorenzo/UWSim/src/Perception_and_Manipulation/manipulation/include;/usr/include;/home/lorenzo/UWSim/devel_isolated/uwsim_bullet/include/uwsim_bullet;/home/lorenzo/UWSim/devel_isolated/uwsim_osgocean/include/uwsim_osgocean " STREQUAL " ")
  set(manipulation_INCLUDE_DIRS "")
  set(_include_dirs "/home/lorenzo/UWSim/src/Perception_and_Manipulation/manipulation/include;/usr/include;/home/lorenzo/UWSim/devel_isolated/uwsim_bullet/include/uwsim_bullet;/home/lorenzo/UWSim/devel_isolated/uwsim_osgocean/include/uwsim_osgocean")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'lorenzo <lorenzo@todo.todo>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${manipulation_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'manipulation' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'manipulation' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/lorenzo/UWSim/src/Perception_and_Manipulation/manipulation/${idir}'.  ${_report}")
    endif()
    _list_append_unique(manipulation_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "manipulation;/usr/lib/x86_64-linux-gnu/libboost_thread.so;-lpthread;/usr/lib/x86_64-linux-gnu/libboost_program_options.so;/usr/lib/x86_64-linux-gnu/libboost_system.so;/usr/lib/x86_64-linux-gnu/libboost_filesystem.so;/usr/lib/x86_64-linux-gnu/libboost_chrono.so;/usr/lib/x86_64-linux-gnu/libboost_date_time.so;/usr/lib/x86_64-linux-gnu/libboost_atomic.so;/home/lorenzo/UWSim/devel_isolated/uwsim_bullet/lib/uwsim_bullet/libBulletCollision.a;/home/lorenzo/UWSim/devel_isolated/uwsim_bullet/lib/uwsim_bullet/libBulletDynamics.a;/home/lorenzo/UWSim/devel_isolated/uwsim_bullet/lib/uwsim_bullet/libBulletSoftBody.a;/home/lorenzo/UWSim/devel_isolated/uwsim_bullet/lib/uwsim_bullet/libLinearMath.a;/usr/lib/x86_64-linux-gnu/libosgSim.so;/usr/lib/x86_64-linux-gnu/libosgUtil.so;/usr/lib/x86_64-linux-gnu/libosgDB.so;/usr/lib/x86_64-linux-gnu/libosgFX.so;/usr/lib/x86_64-linux-gnu/libosgGA.so;/usr/lib/x86_64-linux-gnu/libosgTerrain.so;/usr/lib/x86_64-linux-gnu/libosgViewer.so;/usr/lib/x86_64-linux-gnu/libosgText.so;/usr/lib/x86_64-linux-gnu/libosgWidget.so;/usr/lib/x86_64-linux-gnu/libosg.so;/usr/lib/x86_64-linux-gnu/libosgParticle.so;/usr/lib/x86_64-linux-gnu/libosgManipulator.so;/usr/lib/x86_64-linux-gnu/libOpenThreads.so;/home/lorenzo/UWSim/devel_isolated/uwsim_osgbullet/lib/libosgbCollision.so;/home/lorenzo/UWSim/devel_isolated/uwsim_osgbullet/lib/libosgbDynamics.so;/home/lorenzo/UWSim/devel_isolated/uwsim_osgbullet/lib/libosgbInteraction.so;/home/lorenzo/UWSim/devel_isolated/uwsim_osgocean/lib/libosgOcean.so;/home/lorenzo/UWSim/devel_isolated/uwsim_osgworks/lib/libosgwTools.so;/home/lorenzo/UWSim/devel_isolated/uwsim_osgworks/lib/libosgwQuery.so;/home/lorenzo/UWSim/devel_isolated/uwsim_osgworks/lib/libosgwMx.so")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND manipulation_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND manipulation_LIBRARIES ${library})
  elseif(TARGET ${library})
    list(APPEND manipulation_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND manipulation_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/lorenzo/UWSim/src/Perception_and_Manipulation/manipulation/cmake-build-debug/devel/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(manipulation_LIBRARY_DIRS ${lib_path})
      list(APPEND manipulation_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'manipulation'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND manipulation_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(manipulation_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${manipulation_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "geometry_msgs;image_transport;interactive_markers;kdl_parser;nav_msgs;pcl_ros;pluginlib;resource_retriever;robot_state_publisher;roscpp;rospy;sensor_msgs;tf;underwater_sensor_msgs;urdf;uwsim;underwater_sensor_msgs")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 manipulation_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${manipulation_dep}_FOUND)
      find_package(${manipulation_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${manipulation_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(manipulation_INCLUDE_DIRS ${${manipulation_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(manipulation_LIBRARIES ${manipulation_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${manipulation_dep}_LIBRARIES})
  _list_append_deduplicate(manipulation_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(manipulation_LIBRARIES ${manipulation_LIBRARIES})

  _list_append_unique(manipulation_LIBRARY_DIRS ${${manipulation_dep}_LIBRARY_DIRS})
  list(APPEND manipulation_EXPORTED_TARGETS ${${manipulation_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${manipulation_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
