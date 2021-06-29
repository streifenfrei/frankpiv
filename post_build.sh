if @PYTHON_BINDING@; then
  # copy config file
  cp @CMAKE_CURRENT_SOURCE_DIR@/src/frankpiv/default_config.yml @CATKIN_DEVEL_PREFIX@/@CATKIN_PACKAGE_PYTHON_DESTINATION@/.
  # add __all__ statement to frankpiv __init__.py
  all="__all__ = ['Controller']"
  echo "${all}" >> @CATKIN_DEVEL_PREFIX@/@CATKIN_PACKAGE_PYTHON_DESTINATION@/__init__.py
fi