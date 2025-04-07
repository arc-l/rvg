IF (NOT Matplot++_FOUND)
    SET(Matplot_DIR "extern/matplotplusplus")
    IF (NOT EXISTS ${PROJECT_BINARY_DIR}/Matplot-${CMAKE_BUILD_TYPE}_build)
        EXECUTE_PROCESS(COMMAND mkdir Matplot-${CMAKE_BUILD_TYPE}_build WORKING_DIRECTORY ${PROJECT_BINARY_DIR})
        EXECUTE_PROCESS(COMMAND cmake
                -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
                -DCMAKE_INSTALL_PREFIX=${PROJECT_BINARY_DIR}/Matplot-${CMAKE_BUILD_TYPE}_build
                -DMATPLOTPP_BUILD_SHARED_LIBS=ON
                -DMATPLOTPP_BUILD_EXAMPLES=OFF
                -DMATPLOTPP_BUILD_TESTS=OFF
                -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON
                ${PROJECT_SOURCE_DIR}/${Matplot_DIR}
                WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/Matplot-${CMAKE_BUILD_TYPE}_build)
        EXECUTE_PROCESS(COMMAND make install -j 30 WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/Matplot-${CMAKE_BUILD_TYPE}_build)
    ENDIF ()

    MESSAGE(STATUS "Matplot build directory: ${PROJECT_BINARY_DIR}/Matplot-${CMAKE_BUILD_TYPE}_build")
    FIND_PATH(
            Matplot++_INCLUDE_DIR
            NAMES matplot/matplot.h
            NO_DEFAULT_PATH
            PATHS ${PROJECT_BINARY_DIR}/Matplot-${CMAKE_BUILD_TYPE}_build/include
    )

    FIND_LIBRARY(
            Matplot++_LIBRARY
            NAMES matplot
            NO_DEFAULT_PATH
            PATHS ${PROJECT_BINARY_DIR}/Matplot-${CMAKE_BUILD_TYPE}_build/lib
            NO_CACHE
    )
    FIND_LIBRARY(
            NodeSoup_LIBRARY
            NAMES nodesoup
            NO_DEFAULT_PATH
            PATHS ${PROJECT_BINARY_DIR}/Matplot-${CMAKE_BUILD_TYPE}_build/lib/Matplot++
            NO_CACHE
    )

    MESSAGE(STATUS "Matplot found @ ${Matplot++_INCLUDE_DIR}")
    MESSAGE(STATUS "Matplot found @ ${Matplot++_LIBRARY}")
    MESSAGE(STATUS "NodeSoup found @ ${NodeSoup_LIBRARY}")
    IF (Matplot++_INCLUDE_DIR AND Matplot++_LIBRARY AND NodeSoup_LIBRARY)
        SET(Matplot++_INCLUDE_DIRS ${Matplot++_INCLUDE_DIR})
        SET(Matplot++_LIBRARIES ${Matplot++_LIBRARY} ${NodeSoup_LIBRARY})
        SET(Matplot++_FOUND TRUE)
    ELSE ()
        MESSAGE(FATAL_ERROR "Matplot not found")
    ENDIF ()
    MARK_AS_ADVANCED(Matplot++_FOUND)
    MARK_AS_ADVANCED(Matplot++_INCLUDE_DIR Matplot++_INCLUDE_DIRS)
    MARK_AS_ADVANCED(Matplot++_LIBRARY Matplot++_LIBRARIES)

ENDIF (NOT Matplot++_FOUND)