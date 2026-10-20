"""Build the static RVG Python module and its shared library with CMake."""
import os
import re
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext


class CMakeExtension(Extension):
    def __init__(self, name):
        super().__init__(name, sources=[])
        self.sourcedir = Path(__file__).resolve().parent


class CMakeBuild(build_ext):
    def build_extension(self, ext):
        if shutil.which("cmake") is None:
            raise RuntimeError("CMake is required to build RVG")
        output = Path(self.get_ext_fullpath(ext.name)).resolve().parent
        build = Path(self.build_temp).resolve() / ext.name
        build.mkdir(parents=True, exist_ok=True)
        cache = build / "CMakeCache.txt"
        force = os.environ.get("RVG_FORCE_CMAKE_RECONFIGURE", "").lower() in {
            "1", "true", "on", "yes"
        }
        if cache.exists():
            match = re.search(r"^CMAKE_HOME_DIRECTORY:INTERNAL=(.*)$",
                              cache.read_text(), re.MULTILINE)
            if force or (match and Path(match[1]).resolve() != ext.sourcedir):
                cache.unlink()
                if (build / "CMakeFiles").exists():
                    shutil.rmtree(build / "CMakeFiles")
        configuration = "Debug" if self.debug else "Release"
        arguments = [
            f"-DCMAKE_BUILD_TYPE={configuration}",
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={output}",
            f"-DPython_EXECUTABLE={sys.executable}",
            "-DPYTHON_BINDINGS=ON",
            "-DRVG_BUILD_EXAMPLES=OFF",
            "-DOMPL_ENABLE=OFF",
            "-DBUILD_TESTING=OFF",
        ]
        arguments.extend(shlex.split(os.environ.get("CMAKE_ARGS", "")))
        subprocess.run(["cmake", "-S", str(ext.sourcedir), "-B", str(build), *arguments],
                       check=True)
        subprocess.run(["cmake", "--build", str(build), "--target", "rvg",
                        "--config", configuration, "-j32"], check=True)
        shutil.copyfile(ext.sourcedir / "rvg.pyi", output / "rvg.pyi")
        shutil.copyfile(ext.sourcedir / "py.typed", output / "py.typed")


setup(
    name="rvg",
    version="0.1.0",
    author="Duo Zhang",
    author_email="duo.zhang@rutgers.edu",
    description="Rotation-stacked visibility graphs with exact CPU and CUDA propagation",
    ext_modules=[CMakeExtension("rvg")],
    cmdclass={"build_ext": CMakeBuild},
    zip_safe=False,
    python_requires=">=3.10",
)
