import os
import re
import subprocess
import sys
from pathlib import Path

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext


# A CMakeExtension needs a sourcedir instead of a file list.
# The name must be the _single_ output extension from the CMake build.
# If you need multiple extensions, see scikit-build.
class CMakeExtension(Extension):
    def __init__(self, name: str, sourcedir: str = "") -> None:
        super().__init__(name, sources=[])
        self.sourcedir = os.fspath(Path(sourcedir).resolve())


class CMakeBuild(build_ext):
    def build_extension(self, ext: CMakeExtension) -> None:
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError(
                "CMake must be installed to build the following extensions: , ".join(e.name for e in self.extensions))

        # Must be in this form due to bug in .resolve() only fixed in Python 3.10+
        ext_fullpath = Path.cwd() / self.get_ext_fullpath(ext.name)
        print(f"Path.cwd(): {Path.cwd()}")
        print(f"self.get_ext_fullpath(ext.name): {self.get_ext_fullpath(ext.name)}")
        print(f"ext_fullpath: {ext_fullpath}")
        extdir = ext_fullpath.parent.resolve()
        print(f"extdir: {extdir}")

        # Using this requires trailing slash for auto-detection & inclusion of
        # auxiliary "native" libs

        debug = int(os.environ.get("DEBUG", 0)) if self.debug is None else self.debug
        cfg = "Debug" if debug else "Release"

        # CMake lets you override the generator - we need to check this.
        # Can be set with Conda-Build, for example.
        cmake_generator = os.environ.get("CMAKE_GENERATOR", "")

        # Environment used for CMake configure/build subprocesses.
        cmake_env = os.environ.copy()

        # Set Python_EXECUTABLE instead if you use PYBIND11_FINDPYTHON
        # EXAMPLE_VERSION_INFO shows you how to pass a value into the C++ code
        # from Python.
        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}{os.sep}",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
            "-DPYBIND11_FINDPYTHON=ON",
            f"-DVISIBILITY_QUERY=TEV", # setup the algorithm to do visibility check. Default is Triangle expansion visibility check
            f"-DPYTHON_BINDINGS=ON", # turn on the option to build the python bindings
            "-DBUILD_EXECUTABLES=OFF",
            f"-DCMAKE_BUILD_TYPE={cfg}",  # not used on MSVC, but no harm
        ]
        enable_openmp = os.environ.get("RVG_ENABLE_OPENMP", "ON").upper()
        cmake_args += [f"-DENABLE_OPENMP={enable_openmp}"]

        # On macOS:
        # - OpenMP ON  -> selectable via RVG_OPENMP_TOOLCHAIN:
        #                "clang" (default) or "gcc"
        # - OpenMP OFF -> Apple Clang toolchain (no OpenMP runtime dependency).
        if sys.platform == "darwin":
            if enable_openmp == "ON":
                default_openmp_toolchain = "clang"
                openmp_toolchain = os.environ.get("RVG_OPENMP_TOOLCHAIN", default_openmp_toolchain).lower()
                if openmp_toolchain == "clang":
                    libomp_header = Path("/opt/homebrew/opt/libomp/include/omp.h")
                    libomp_lib = Path("/opt/homebrew/opt/libomp/lib/libomp.dylib")
                    if not libomp_header.exists() or not libomp_lib.exists():
                        raise RuntimeError(
                            "RVG_OPENMP_TOOLCHAIN=clang requires Homebrew libomp, but it was not found at "
                            "/opt/homebrew/opt/libomp. Install it with `brew install libomp`, or use "
                            "`RVG_OPENMP_TOOLCHAIN=gcc`."
                        )
                    cmake_args += [
                        "-DCMAKE_C_COMPILER=/usr/bin/clang",
                        "-DCMAKE_CXX_COMPILER=/usr/bin/clang++",
                        "-DOPENMP_CXX_FLAG=-Xpreprocessor -fopenmp -I/opt/homebrew/opt/libomp/include",
                        "-DOPENMP_EXE_LIB=/opt/homebrew/opt/libomp/lib/libomp.dylib",
                        "-DOPENMP_LINK_FLAGS=-Wl,-rpath,/opt/homebrew/opt/libomp/lib",
                    ]
                    # Auto-provide Homebrew libomp flags so users do not need to export them manually.
                    cppflags = cmake_env.get("CPPFLAGS", "")
                    ldflags = cmake_env.get("LDFLAGS", "")
                    libomp_cpp = "-I/opt/homebrew/opt/libomp/include"
                    libomp_ld = "-L/opt/homebrew/opt/libomp/lib"
                    if libomp_cpp not in cppflags:
                        cmake_env["CPPFLAGS"] = (cppflags + " " + libomp_cpp).strip()
                    if libomp_ld not in ldflags:
                        cmake_env["LDFLAGS"] = (ldflags + " " + libomp_ld).strip()
                else:
                    cmake_args += [
                        "-DCMAKE_C_COMPILER=/opt/homebrew/bin/gcc-11",
                        "-DCMAKE_CXX_COMPILER=/opt/homebrew/bin/g++-11",
                    ]
            else:
                cmake_args += [
                    "-DCMAKE_C_COMPILER=/usr/bin/clang",
                    "-DCMAKE_CXX_COMPILER=/usr/bin/clang++",
                ]
        build_args = ["--verbose", "-j 24"]
        # Adding CMake arguments set as environment variable
        # (needed e.g. to build for ARM OSx on conda-forge)
        if "CMAKE_ARGS" in os.environ:
            cmake_args += [item for item in os.environ["CMAKE_ARGS"].split(" ") if item]

        # In this example, we pass in the version to C++. You might not need to.
        cmake_args += [f"-DEXAMPLE_VERSION_INFO={self.distribution.get_version()}"]

        if self.compiler.compiler_type != "msvc":
            # Using Ninja-build since it a) is available as a wheel and b)
            # multithreads automatically. MSVC would require all variables be
            # exported for Ninja to pick it up, which is a little tricky to do.
            # Users can override the generator with CMAKE_GENERATOR in CMake
            # 3.15+.
            if not cmake_generator or cmake_generator == "Ninja":
                try:
                    import ninja

                    ninja_executable_path = Path(ninja.BIN_DIR) / "ninja"
                    cmake_args += [
                        "-GNinja",
                        f"-DCMAKE_MAKE_PROGRAM:FILEPATH={ninja_executable_path}",
                    ]
                except ImportError:
                    pass


        # Set CMAKE_BUILD_PARALLEL_LEVEL to control the parallel build level
        # across all generators.
        if "CMAKE_BUILD_PARALLEL_LEVEL" not in os.environ:
            # self.parallel is a Python 3 only way to set parallel jobs by hand
            # using -j in the build_ext call, not supported by pip or PyPA-build.
            if hasattr(self, "parallel") and self.parallel:
                # CMake 3.12+ only.
                build_args += [f"-j{self.parallel}"]

        # Use a stable build directory so repeated `pip install -e .` runs are incremental.
        # Can be overridden with RVG_PIP_BUILD_DIR.
        default_build_root = Path.cwd() / "build" / "pip-cmake"
        build_root = Path(os.environ.get("RVG_PIP_BUILD_DIR", str(default_build_root)))
        build_key = f"{ext.name}-{cfg}-openmp-{enable_openmp.lower()}"
        build_temp = build_root / build_key
        if not build_temp.exists():
            build_temp.mkdir(parents=True)

        subprocess.run(
            ["cmake", ext.sourcedir, *cmake_args], cwd=build_temp, check=True, env=cmake_env
        )
        subprocess.run(
            ["cmake", "--build", ".", *build_args], cwd=build_temp, check=True, env=cmake_env
        )


# The information here can also be placed in setup.cfg - better separation of
# logic and declaration, and simpler if you include description/version in a file.
setup(
    name="rvg",
    version="0.0.2",
    author="Duo Zhang",
    author_email="duo.zhang@rutgers.edu",
    description="A test project using pybind11 and CMake",
    long_description="",
    ext_modules=[CMakeExtension("rvg")],
    cmdclass={"build_ext": CMakeBuild},
    zip_safe=False,
    python_requires=">=3.7",
    package_data = {
        'rvg': ['*.pyi', "py.typed"],
    },
)
