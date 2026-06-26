# Copyright 2020 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# Check if there is C++ code at all
########################################################################
if(NOT lidarModule_sources)
    message(STATUS "No C++ sources... skipping python bindings")
    return()
endif(NOT lidarModule_sources)

########################################################################
# Check for pygccxml
########################################################################
gr_python_check_module_raw("pygccxml" "import pygccxml" PYGCCXML_FOUND)

include(GrPybind)

########################################################################
# Python Bindings
########################################################################
list(APPEND lidarModule_python_files
    OFDM_Packetizer_python.cc
    OFDM_Detector_python.cc
    OFDM_Demod_python.cc
    Velo_to_XYZ_python.cc python_bindings.cc)

gr_pybind_make_oot(lidarModule ../../.. gr::lidarModule "${lidarModule_python_files}")

# copy bindings extension for use in QA test module
add_custom_command(
    TARGET lidarModule_python
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:lidarModule_python>
            ${PROJECT_BINARY_DIR}/test_modules/gnuradio/lidarModule/)

install(
    TARGETS lidarModule_python
    DESTINATION ${GR_PYTHON_DIR}/gnuradio/lidarModule
    COMPONENT pythonapi)
