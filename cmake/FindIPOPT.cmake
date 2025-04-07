#BLAS/LAPACK
FIND_PACKAGE(BLAS QUIET)
FIND_PACKAGE(LAPACK QUIET)
IF(BLAS_FOUND AND LAPACK_FOUND)
    MESSAGE(STATUS "Found BLAS/LAPACK @ ${BLAS_LIBRARIES};${LAPACK_LIBRARIES}")
    LIST(APPEND ALL_LIBRARIES ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES})
ELSE(BLAS_FOUND AND LAPACK_FOUND)
    MESSAGE(SEND_ERROR "Cannot find BLAS/LAPACK!")
ENDIF(BLAS_FOUND AND LAPACK_FOUND)

#Ipopt
IF(MSVC)
    OPTION(NO_IPOPT_SUPPORT "Whether to support IPOPT solver" ON)
ELSE(MSVC)
    OPTION(NO_IPOPT_SUPPORT "Whether to support IPOPT solver" OFF)
ENDIF(MSVC)
IF(NOT NO_IPOPT_SUPPORT)
    SET(IPOPT_ROOT "extern/Ipopt-3.12.11")
    IF(NOT EXISTS ${PROJECT_BINARY_DIR}/Ipopt-build)
        EXECUTE_PROCESS(COMMAND mkdir Ipopt-build WORKING_DIRECTORY ${PROJECT_BINARY_DIR})
        EXECUTE_PROCESS(COMMAND bash ${PROJECT_SOURCE_DIR}/${IPOPT_ROOT}/configure
                --prefix=${PROJECT_BINARY_DIR}/Ipopt-build --enable-debug=yes --enable-shared=no --with-pic=yes
                WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/Ipopt-build)
        EXECUTE_PROCESS(COMMAND make install WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/Ipopt-build)
    ENDIF()
    FIND_LIBRARY(IPOPT_LIBRARY NAMES ipopt Ipopt libipopt libIpopt PATHS
            ${PROJECT_BINARY_DIR}/Ipopt-build/lib
            ${PROJECT_BINARY_DIR}/Ipopt-build/Ipopt/src/Interfaces/.libs NO_DEFAULT_PATH)
    FIND_LIBRARY(COINHSL_LIBRARY NAMES coinhsl libcoinhsl PATHS
            ${PROJECT_BINARY_DIR}/Ipopt-build/lib
            ${PROJECT_BINARY_DIR}/Ipopt-build/ThirdParty/HSL/.libs NO_DEFAULT_PATH)
    FIND_LIBRARY(GFORTRAN_LIBRARY NAMES gfortran libgfortran PATHS
            /usr/lib/gcc/x86_64-linux-gnu/*
            /usr/lib/x86_64-linux-gnu/*
            /usr/lib/gcc/x86_64-linux-gnu
            /usr/lib/x86_64-linux-gnu)
    IF(IPOPT_LIBRARY AND GFORTRAN_LIBRARY AND COINHSL_LIBRARY)
        INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR}/Ipopt-build/include)
        MESSAGE(STATUS "Found Ipopt Libraries @ ${IPOPT_LIBRARY} ${COINHSL_LIBRARY} ${GFORTRAN_LIBRARY}")
        LIST(APPEND ALL_LIBRARIES ${COINHSL_LIBRARY} ${GFORTRAN_LIBRARY} ${CMAKE_DL_LIBS})
        LIST(APPEND ALL_STATIC_LIBRARIES ${IPOPT_LIBRARY})
        ADD_DEFINITIONS(-DIPOPT_SUPPORT)
    ELSE()
        MESSAGE(WARNING "Cannot find GFortran or Ipopt, compiling without it!")
        INCLUDE("${PROJECT_SOURCE_DIR}/cmake/FindCoinHSL.cmake")
    ENDIF()
ENDIF()
