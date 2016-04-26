@[if DEVELSPACE]@
list(APPEND @(PROJECT_NAME)_INCLUDE_DIRS @(CMAKE_CURRENT_SOURCE_DIR))
@[else]@
get_filename_component(@(PROJECT_NAME)_inc_temp ${@(PROJECT_NAME)_DIR}/../../../@(CATKIN_PACKAGE_INCLUDE_DESTINATION)/include REALPATH)
list(APPEND @(PROJECT_NAME)_INCLUDE_DIRS ${@(PROJECT_NAME)_inc_temp})

# this is necessary because headers in 'qt_property_browser' itself are included without the 'qt_property_browser' prefix
list(APPEND @(PROJECT_NAME)_INCLUDE_DIRS ${@(PROJECT_NAME)_inc_temp}/qt_property_browser)

unset(@(PROJECT_NAME)_inc_temp)
@[end if]@
