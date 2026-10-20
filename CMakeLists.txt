cmake_minimum_required(VERSION 3.18)
project(RotationalVisibilityGraph VERSION 0.1.0 LANGUAGES C CXX)

option(PYTHON_BINDINGS "Build the static RVG Python module" OFF)
option(RVG_BUILD_EXAMPLES "Build static RVG example and benchmark executables" ON)
option(RVG_BUILD_DEBUG_TOOLS "Build the legacy interactive geometry tools" OFF)
option(OMPL_ENABLE "Build the original static RVG/OMPL comparisons" OFF)
option(TRACY_ENABLE "Enable Tracy instrumentation" OFF)
option(USE_CXX11_ABI "Use the C++11 standard-library ABI" ON)
set(RVG_CUDA "AUTO" CACHE STRING "CUDA vertex propagation: AUTO, ON, or OFF")
set_property(CACHE RVG_CUDA PROPERTY STRINGS AUTO ON OFF)
string(TOUPPER "${RVG_CUDA}" RVG_CUDA_MODE)
if(NOT RVG_CUDA_MODE MATCHES "^(AUTO|ON|OFF)$")
    message(FATAL_ERROR "RVG_CUDA must be AUTO, ON, or OFF")
endif()
if(USE_IPOPT)
    message(FATAL_ERROR "IPOPT rotation optimization is no longer supported")
endif()
if(DEFINED VISIBILITY_QUERY AND NOT VISIBILITY_QUERY STREQUAL "TEV")
    message(FATAL_ERROR "Static RVG uses triangular-expansion visibility (TEV)")
endif()
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(CTest)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization filesystem thread chrono)
find_package(OpenMP REQUIRED COMPONENTS CXX)
find_package(TINYXML2 REQUIRED)
set(CGAL_WITH_GMPXX OFF)
set(WITH_GMPXX OFF)
find_package(CGAL CONFIG QUIET COMPONENTS Core)
if(NOT CGAL_FOUND)
    set(CGAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/extern/CGAL/Installation/lib/cmake/CGAL")
    find_package(CGAL CONFIG REQUIRED COMPONENTS Core)
endif()
if(DEFINED PYTHON_EXECUTABLE AND NOT DEFINED Python_EXECUTABLE)
    set(Python_EXECUTABLE "${PYTHON_EXECUTABLE}")
endif()
find_package(Python QUIET COMPONENTS Interpreter)
if(Python_Interpreter_FOUND)
    set(RVG_PYTHON "${Python_EXECUTABLE}")
else()
    set(RVG_PYTHON "python3")
endif()

add_library(RotationalVisibilityGraph SHARED
    VisibilityGraph/Vertex.cpp
    VisibilityGraph/Edge.cpp
    VisibilityGraph/Polygon.cpp
    VisibilityGraph/Graph.cpp
    VisibilityGraph/Layer.cpp
    VisibilityGraph/ConfigurationSpaceLayer.cpp
    VisibilityGraph/VisibilityGraph.cpp
    VisibilityGraph/VertexPropagation.cpp
    VisibilityGraph/VertexPropagationFixedPoint.cpp
    VisibilityGraph/MapGenerator.cpp
    VisibilityGraph/RectangleObstacleMapGenerator.cpp
    Utils/Utils.cpp
)
target_include_directories(RotationalVisibilityGraph PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}" ${TINYXML2_INCLUDE_DIRS})
target_link_libraries(RotationalVisibilityGraph PUBLIC
    CGAL::CGAL CGAL::CGAL_Core Eigen3::Eigen OpenMP::OpenMP_CXX
    Boost::serialization Boost::filesystem Boost::thread Boost::chrono
    ${TINYXML2_LIBRARIES})
target_compile_definitions(RotationalVisibilityGraph PUBLIC
    PYTHON_EXECUTABLE="${RVG_PYTHON}")
if(NOT USE_CXX11_ABI)
    target_compile_definitions(RotationalVisibilityGraph PUBLIC _GLIBCXX_USE_CXX11_ABI=0)
endif()
target_compile_options(RotationalVisibilityGraph PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wno-unused-local-typedefs>)

set(RVG_CUDA_ENABLED OFF)
if(NOT RVG_CUDA_MODE STREQUAL "OFF")
    include(CheckLanguage)
    check_language(CUDA)
    if(CMAKE_CUDA_COMPILER)
        enable_language(CUDA)
        find_package(CUDAToolkit QUIET)
        if(CUDAToolkit_FOUND)
            set(RVG_CUDA_ENABLED ON)
            target_sources(RotationalVisibilityGraph PRIVATE VisibilityGraph/VertexPropagationCuda.cu)
            target_compile_definitions(RotationalVisibilityGraph PUBLIC RVG_HAS_CUDA=1)
            target_link_libraries(RotationalVisibilityGraph PUBLIC CUDA::cudart)
            set_target_properties(RotationalVisibilityGraph PROPERTIES
                CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED ON
                CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}")
        endif()
    endif()
    if(RVG_CUDA_MODE STREQUAL "ON" AND NOT RVG_CUDA_ENABLED)
        message(FATAL_ERROR "RVG_CUDA=ON requires a CUDA compiler and toolkit")
    endif()
endif()
message(STATUS "CUDA vertex propagation: ${RVG_CUDA_ENABLED}")

if(TRACY_ENABLE)
    set(TRACY_ON_DEMAND ON CACHE BOOL "Connect Tracy only on demand")
    add_subdirectory(extern/tracy EXCLUDE_FROM_ALL)
    set_target_properties(TracyClient PROPERTIES POSITION_INDEPENDENT_CODE ON)
    target_link_libraries(RotationalVisibilityGraph PUBLIC Tracy::TracyClient)
endif()

function(rvg_executable name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE RotationalVisibilityGraph)
endfunction()
if(RVG_BUILD_EXAMPLES)
    foreach(name mainRVGConfigRun mainMapGeneration mainBenchmarkVertexPropagation
                 mainDrawMap mainWeights mainDrawConvergence)
        rvg_executable(${name} Main/Experiments/${name}.cpp)
    endforeach()
    target_compile_definitions(mainBenchmarkVertexPropagation PRIVATE
        RVG_HARD_BENCHMARK_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/Configs/HardBenchmarks")
endif()
if(RVG_BUILD_DEBUG_TOOLS)
    file(GLOB RVG_DEBUG_SOURCES CONFIGURE_DEPENDS Main/VisibilityGraphDebug/*.cpp)
    foreach(source IN LISTS RVG_DEBUG_SOURCES)
        get_filename_component(name "${source}" NAME_WE)
        rvg_executable(${name} "${source}")
    endforeach()
endif()

if(OMPL_ENABLE)
    include(LoadOMPL)
    add_library(rvg_comparisons STATIC OMPL/OMPLEnv.cpp OMPL/CollisionDetection.cpp)
    target_link_libraries(rvg_comparisons PUBLIC RotationalVisibilityGraph rvg_ompl)
    foreach(name mainRVG_OMPL_Comparison mainRVG_OMPL_ComparisonNarrowPassage
                 mainOMPLNarrowPassageTest mainOMPLConfigRun mainOMPLConvergenceTest mainMultiQuery)
        add_executable(${name} Main/Experiments/${name}.cpp)
        target_link_libraries(${name} PRIVATE rvg_comparisons)
    endforeach()
endif()

if(PYTHON_BINDINGS)
    find_package(Python REQUIRED COMPONENTS Interpreter Development)
    set(PYBIND11_FINDPYTHON ON)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/extern/pybind11/CMakeLists.txt")
        add_subdirectory(extern/pybind11 EXCLUDE_FROM_ALL)
    else()
        find_package(pybind11 CONFIG REQUIRED)
    endif()
    pybind11_add_module(rvg Python/bindings.cpp)
    target_link_libraries(rvg PRIVATE RotationalVisibilityGraph)
    if(APPLE)
        set(RVG_PYTHON_RPATH "@loader_path")
    else()
        set(RVG_PYTHON_RPATH "$ORIGIN")
    endif()
    set_target_properties(rvg PROPERTIES
        BUILD_WITH_INSTALL_RPATH ON INSTALL_RPATH "${RVG_PYTHON_RPATH}")
endif()

if(BUILD_TESTING)
    foreach(name testGraphMerge testConfigurationSpaceLayer testMapGenerator testSolvableMapBenchmark)
        rvg_executable(${name} Tests/VisibilityGraph/${name}.cpp)
        add_test(NAME ${name} COMMAND ${name})
        set_tests_properties(${name} PROPERTIES TIMEOUT 120)
    endforeach()
    target_compile_definitions(testSolvableMapBenchmark PRIVATE
        RVG_BASELINE_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/Configs/Baselines")
    rvg_executable(testVertexPropagation Tests/VisibilityGraph/testVertexPropagation.cpp)
    target_compile_definitions(testVertexPropagation PRIVATE
        RVG_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/Configs")
    add_test(NAME testVertexPropagationCpu COMMAND testVertexPropagation)
    set_tests_properties(testVertexPropagationCpu PROPERTIES TIMEOUT 180)
    if(RVG_CUDA_ENABLED)
        add_test(NAME testVertexPropagationCuda COMMAND testVertexPropagation --cuda)
        set_tests_properties(testVertexPropagationCuda PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 180)
    endif()
    if(PYTHON_BINDINGS)
        add_test(NAME testPythonBindings COMMAND ${CMAKE_COMMAND} -E env
            "PYTHONPATH=$<TARGET_FILE_DIR:rvg>" ${Python_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/Tests/Python/test_bindings.py)
    endif()
endif()
