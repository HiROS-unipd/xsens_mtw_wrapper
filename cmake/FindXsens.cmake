find_path(Xsens_INCLUDE_DIR xsensdeviceapi.h
          HINTS "$ENV{Xsens_DIR}"
          PATH_SUFFIXES "include" "xsens/include")

find_library(Xsens_xsensdeviceapi_LIBRARY xsensdeviceapi
             HINTS "$ENV{Xsens_DIR}"
             PATH_SUFFIXES "lib" "xsens/lib")

find_library(Xsens_xstypes_LIBRARY xstypes
             HINTS "$ENV{Xsens_DIR}"
             PATH_SUFFIXES "lib" "xsens/lib")

mark_as_advanced(Xsens_INCLUDE_DIR
                 Xsens_xsensdeviceapi_LIBRARY
                 Xsens_xstypes_LIBRARY)

if(Xsens_xsensdeviceapi_LIBRARY AND Xsens_xstypes_LIBRARY AND Xsens_INCLUDE_DIR)
    set(Xsens_LIBRARIES
        "${Xsens_xstypes_LIBRARY};${Xsens_xsensdeviceapi_LIBRARY}")
    set(Xsens_INCLUDE_DIRS "${Xsens_INCLUDE_DIR}")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Xsens
                                  DEFAULT_MSG
                                  Xsens_LIBRARIES
                                  Xsens_INCLUDE_DIRS)

if(COMMAND set_package_properties)
    set_package_properties(Xsens
                           PROPERTIES
                           DESCRIPTION
                           "API for Xsens MTw Awinda"
                           URL
                           "https://www.xsens.com/products/mtw-awinda/")
endif()
