<div align="center">
<h1>Asymptotically-Optimal Multi-Query Path Planning for Moving A Convex Polygon in 2D</h1>
<h3>
<a href="https://duozhangrobotics.github.io/" target="_blank"><nobr>Duo Zhang</nobr></a> &emsp;
<nobr>Zihe Ye</nobr> &emsp;
<a href="https://arc-l.github.io/group.html" target="_blank"><nobr>Jingjin Yu</nobr></a>
</h3>
</div>


The paper link [https://arxiv.org/pdf/2409.03920](https://arxiv.org/pdf/2409.03920) (ICRA 2025).

## Clone this repository with submodules
```bash
git clone --recurse-submodules https://github.com/arc-l/rvg.git 
```

If you have already cloned the repository, you can run the following command to get the submodules
```bash
git submodule update --init --recursive
```

## Configure python for RVG
In RVG, we use matplotlib to do all the visualization in the C++ code. There are two ways of setting up python for RVG:
1. Setup the CMAKE variable `PYTHON_EXECUTABLE`
   If you have a specific python that you want to use, just run cmake with
   ```bash
    cmake -DPYTHON_EXECUTABLE=<path-to-your-python>
   ```
   But you have to make sure libpython is already installed for the specific python. Or if you don't want to copy the path to your python and you have already set up a python environment in your terminal, you can simply run:
   ```bash
   cmake -DPYTHON_EXECUTABLE=$(which python)
   ```
2. Virtual Environment (**Recommended**)
    To have a clean python environment, create a virtual environment and activate
    ```bash
    conda create -n rvg python==3.9 numpy matplotlib # matplotlib is for visualizing the solutions and environments.
    conda install -c conda-forge cpython # for different versions of libpython
    conda activate rvg
    ```
    Then you can simply run `cmake` without setting the `PYTHON_EXECUTABLE` because cmake will use the python in the conda environment by default.

## Requirements
For Ubuntu
```bash
sudo apt install libboost-all-dev  libgmp-dev  libmpfr-dev libtinyxml2-dev libeigen3-dev
```

## To build c++ library
```bash
mkdir build
cd build
cmake ..
```

## To build python package
```bash
# activate the environment you want
pip install -e .
```

## To build the function annotation if you have built more functions
1. Install pybind11-stubgen first. 
    ```bash
    pip install pybind11-stubgen
    ```
2. Generate the stub file
    ```bash
    pybind11-stubgen -o ./ rvg
    ```
    This will generate a `rvg.pyi` file in the project folder so the function signitures can be found be python intellisense.

