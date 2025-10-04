###########################################################
# SmartSpectra_dependencies.cmake
# Created by Greg on 9/3/2024.
# Copyright (C) 2024 Presage Security, Inc.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program; if not, write to the Free Software Foundation,
# Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
###########################################################
if (NOT SMART_SPECTRA_LOCAL_BUILD)
    find_package(PhysiologyEdge REQUIRED)
endif ()

if (LINUX)
    find_package(V4L REQUIRED)
endif ()

# Optional: Pylon SDK for Basler cameras
find_package(PkgConfig QUIET)
if (PkgConfig_FOUND)
    pkg_check_modules(PYLON QUIET pylon)
endif ()

if (NOT PYLON_FOUND)
    # Try to find Pylon manually by looking for common installation paths
    find_path(PYLON_INCLUDE_DIR
        NAMES pylon/PylonIncludes.h
        PATHS 
            /opt/pylon/include
            /usr/local/include
            /usr/include
        DOC "Pylon SDK include directory"
    )
    
    find_library(PYLON_BASE_LIBRARY
        NAMES pylonbase
        PATHS
            /opt/pylon/lib64
            /opt/pylon/lib
            /usr/local/lib64
            /usr/local/lib
            /usr/lib64
            /usr/lib
        DOC "Pylon base library"
    )
    
    if (PYLON_INCLUDE_DIR AND PYLON_BASE_LIBRARY)
        set(PYLON_FOUND TRUE)
        set(PYLON_INCLUDE_DIRS ${PYLON_INCLUDE_DIR})
        set(PYLON_LIBRARIES ${PYLON_BASE_LIBRARY})
        
        # Find additional required Pylon libraries
        find_library(PYLON_UTILITY_LIBRARY NAMES pylonutility PATHS ${PYLON_BASE_LIBRARY}/../.. PATH_SUFFIXES lib lib64)
        find_library(PYLON_GIGE_LIBRARY NAMES pylongige PATHS ${PYLON_BASE_LIBRARY}/../.. PATH_SUFFIXES lib lib64)
        find_library(PYLON_USB_LIBRARY NAMES pylonusb PATHS ${PYLON_BASE_LIBRARY}/../.. PATH_SUFFIXES lib lib64)
        
        # Find required GenICam libraries
        find_library(GCBASE_LIBRARY NAMES GCBase_gcc_v3_1_Basler_pylon_v3 GCBase_gcc_v3_1_Basler_pylon PATHS ${PYLON_BASE_LIBRARY}/../.. PATH_SUFFIXES lib lib64)
        find_library(GENAPI_LIBRARY NAMES GenApi_gcc_v3_1_Basler_pylon_v3 GenApi_gcc_v3_1_Basler_pylon PATHS ${PYLON_BASE_LIBRARY}/../.. PATH_SUFFIXES lib lib64)
        find_library(LOG4CPP_LIBRARY NAMES log4cpp_gcc_v3_1_Basler_pylon_v3 log4cpp_gcc_v3_1_Basler_pylon PATHS ${PYLON_BASE_LIBRARY}/../.. PATH_SUFFIXES lib lib64)
        
        if (PYLON_UTILITY_LIBRARY)
            list(APPEND PYLON_LIBRARIES ${PYLON_UTILITY_LIBRARY})
        endif ()
        if (PYLON_GIGE_LIBRARY)
            list(APPEND PYLON_LIBRARIES ${PYLON_GIGE_LIBRARY})
        endif ()
        if (PYLON_USB_LIBRARY)
            list(APPEND PYLON_LIBRARIES ${PYLON_USB_LIBRARY})
        endif ()
        if (GCBASE_LIBRARY)
            list(APPEND PYLON_LIBRARIES ${GCBASE_LIBRARY})
        endif ()
        if (GENAPI_LIBRARY)
            list(APPEND PYLON_LIBRARIES ${GENAPI_LIBRARY})
        endif ()
        if (LOG4CPP_LIBRARY)
            list(APPEND PYLON_LIBRARIES ${LOG4CPP_LIBRARY})
        endif ()
    endif ()
endif ()

if (PYLON_FOUND)
    message(STATUS "Found Pylon SDK: ${PYLON_INCLUDE_DIRS}")
    set(HAVE_PYLON_SDK ON)
else ()
    message(STATUS "Pylon SDK not found. Basler camera support will be disabled.")
    set(HAVE_PYLON_SDK OFF)
endif ()

# PhysiologyEdge does not support GPU on Apple machines yet
if (NOT APPLE AND ENABLE_GPU)
    find_package(OpenGL REQUIRED OpenGL GLES3)
endif ()

if (BUILD_TESTS)
    if (USE_SYSTEM_CATCH2)
        find_package(Catch2)
        if (TARGET Catch2::Catch2)
            message(STATUS "Using installed third-party library Catch2")
            set(CATCH2_TARGET "Catch2::Catch2")
        else ()
            message(STATUS "Unable to find third-party library Catch2 installed on system.
            Setting USE_SYSTEM_CATCH2 to OFF and building from source instead.")
            set(USE_SYSTEM_CATCH2 OFF)
        endif ()
    endif ()
    if (NOT USE_SYSTEM_CATCH2)
        include(FetchContent)
        FetchContent_Declare(
                Catch2
                GIT_REPOSITORY https://github.com/catchorg/Catch2.git
                GIT_TAG v3.7.0
        )
        FetchContent_MakeAvailable(Catch2)
        list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/contrib)
        include(CTest)
        include(Catch)
        set(CATCH2_TARGET "Catch2::Catch2")
    endif ()
endif ()
