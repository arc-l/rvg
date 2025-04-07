IF(NOT QGLViewer_FOUND)

    FIND_PATH(QGLViewer_INCLUDE_DIR QGLViewer/qglviewer.h
            /usr/include
            /usr/local/include
    )

    MESSAGE(STATUS "QGLViewer_INCLUDE_DIR: ${QGLViewer_INCLUDE_DIR}")

    FIND_LIBRARY(QGLViewer_LIBRARY NAMES QGLViewer-qt5 PATHS
            ${LIB_INSTALL_DIR}
#            /usr/lib/x86_64-linux-gnu
            NO_CACHE)
    MESSAGE(STATUS "QGLViewer_LIBRARY: ${QGLViewer_LIBRARY}")

    IF(QGLViewer_INCLUDE_DIR AND QGLViewer_LIBRARY)
        SET(QGLViewer_FOUND TRUE)
    ENDIF(QGLViewer_INCLUDE_DIR AND QGLViewer_LIBRARY)

    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args(QGLViewer DEFAULT_MSG
            QGLViewer_INCLUDE_DIR QGLViewer_LIBRARY)

ENDIF(NOT QGLViewer_FOUND)