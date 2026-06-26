find_package(PkgConfig)

PKG_CHECK_MODULES(PC_GR_LIDARMODULE gnuradio-lidarModule)

FIND_PATH(
    GR_LIDARMODULE_INCLUDE_DIRS
    NAMES gnuradio/lidarModule/api.h
    HINTS $ENV{LIDARMODULE_DIR}/include
        ${PC_LIDARMODULE_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    GR_LIDARMODULE_LIBRARIES
    NAMES gnuradio-lidarModule
    HINTS $ENV{LIDARMODULE_DIR}/lib
        ${PC_LIDARMODULE_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          )

include("${CMAKE_CURRENT_LIST_DIR}/gnuradio-lidarModuleTarget.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GR_LIDARMODULE DEFAULT_MSG GR_LIDARMODULE_LIBRARIES GR_LIDARMODULE_INCLUDE_DIRS)
MARK_AS_ADVANCED(GR_LIDARMODULE_LIBRARIES GR_LIDARMODULE_INCLUDE_DIRS)
