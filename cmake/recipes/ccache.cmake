# -----------------------------------------------------------------------------
# CPM and CCache specific options
# -----------------------------------------------------------------------------
include(cmake/CPM.cmake)
CPMAddPackage(
  NAME Ccache.cmake
  GITHUB_REPOSITORY TheLartians/Ccache.cmake
  VERSION 1.2
)
