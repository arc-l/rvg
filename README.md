## Asymptotically-Optimal Multi-Query Path Planning for Moving A Convex Polygon in 2D

<a href="https://duozhangrobotics.github.io/" target="_blank"><nobr>Duo Zhang</nobr></a> &emsp;
<nobr>Zihe Ye</nobr> &emsp;
<a href="https://arc-l.github.io/group.html" target="_blank"><nobr>Jingjin Yu</nobr></a> &emsp; (Accepted to [ICRA 2025](https://2025.ieee-icra.org/))


[![YouTube](https://badges.aleen42.com/src/youtube.svg)](https://youtu.be/) [![Static Badge](https://img.shields.io/badge/RVG-arXiv-blue )](https://arxiv.org/pdf/2409.03920)

<div align="center">
<p float="left">
  <img src="https://github.com/user-attachments/assets/ee9ec0a4-6ca6-401d-a9cb-382bc864adf2" width="30%" />
  <img src="https://github.com/user-attachments/assets/5a092d9f-583f-4b50-ac1b-6ad1290a553a" width="30%" />
  <img src="https://github.com/user-attachments/assets/110dd7d3-0de2-4bd9-8e55-c0b13f61f0cd" width="30%" />
</p>
</div>

## Table of Contents
- [Introduction](#intro)
  - [What Does a Rotation-Stacked Visibility Graph (RVG) Do?](#rvg)
  - [Comparisons with SOTA SE(2) Rigid Body Planners](#compare)
  - [Additional Examples](#examples)
- [Installation and Example Usage](#install)
  - [C++]()
  - [Python]()
- [Citation](#citation)

## <a name="intro"></a>Introduction
### <a name="rvg"></a>What does a rotation-stacked visibility graph (RVG) do?  

### <a name="compare"></a>Comparisons with SOTA SE(2) Rigid Body Planners

### <a name="examples"></a>Additional Examples
![pacman](https://github.com/user-attachments/assets/c2239184-de8a-4a9e-aa32-11d4e3ccfdae)
![corridor](https://github.com/user-attachments/assets/fb196e46-af34-4cd1-9a8f-d797062ffe30)



https://github.com/user-attachments/assets/71e543a1-9535-4a94-8db5-5ecd59227703


https://github.com/user-attachments/assets/e8292aab-cb14-4794-8171-dd2f7ae271db




## <a name="install"></a>Installation and Example Usage
### Clone this repository with submodules
```bash
git clone --recurse-submodules https://github.com/arc-l/rvg.git 
```

If you have already cloned the repository, you can run the following command to get the submodules
```bash
git submodule update --init --recursive
```

### Requirements
For Ubuntu
```bash
sudo apt install libboost-all-dev  libgmp-dev  libmpfr-dev libtinyxml2-dev libeigen3-dev
```
For Mac, use brew to install the corresponding libraries, and the C++ code should be able to run. The python-bindings for MacOS is still under development.

### Configure python for RVG
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
    To have a clean python environment, create a

https://github.com/user-attachments/assets/5948e0ff-252d-46ea-9b63-699d5e7fa5a9

 virtual environment and activate
    ```bash
    conda create -n rvg python==3.9 numpy matplotlib # matplotlib is for visualizing the solutions and environments.
    conda install -c conda-forge cpython # for different versions of libpython
    conda activate rvg
    ```
    Then you can simply run `cmake` without setting the `PYTHON_EXECUTABLE` because cmake will use the python in the conda environment by default.


### To build python package
```bash
# activate the environment you want
pip install -e .
```

### To build the C++ code
There are a few CMake variables that can be configured:
1. `PYTHON_EXECUTABLE`: see [here](#configure-python-for-rvg)
2. `TRACY_ENABLE`: To use [tracy](https://github.com/wolfpld/tracy) to profile the performance
3. `OMPL_ENABLE`: OMPL will be dowloaded and installed, if you want to run comparisons between sampling based motion planner with RVG
```
mkdir build
cd build
cmake -D<settings of variables> ..
make -j24
```

### To build the function annotation for python if you have built more functions with Pybind11
1. Install pybind11-stubgen first. 
    ```bash
    pip install pybind11-stubgen
    ```
2. Generate the stub file
    ```bash
    pybind11-stubgen -o ./ rvg
    ```
    This will generate a `rvg.pyi` file in the project folder so the function signitures can be found be python intellisense. You will get some errors on the symbols/functions from other Third-Party which is not specified in pybind11. It won't affect the other stubs.

## Usage
### Run the code in Python
Once RVG is installed, try to import it:
```python
from rvg import vertex, polygon, visibility_graph
```
Then you can create a vertex given its `x` and `y`
```python
v1 = vertex(0, 0)
```
You can also create a polygon given a list of vertices:
```python
p1 = polygon( [
        vertex(0, 0),
        vertex(2, 0),
        vertex(2, 2),
        vertex(0, 2)
    ])
```
Then you can construct a RVG and find the shortest path from `start(vertex)` to `goal(vertex)`:
```python=
vg = visibility_graph(robot=robot, # represented by a polygon
                      border = border, # represented by a polygon
                      obstacles = obstacles, # represented by a list of polygons 
                      resolution=18, 
                      considerSymmetry=True, # whether to consider the symmetry of the robot geometry
                      hashWithTheta=True, 
                      numThreads=1,  # number of cores for parallelization
                      verbose=False, # show running details
                      fineApprox=True # Use a finer approximation of the rotation range
                      )
path = vg.shortestPath(start, goal) # search for the shortest path
```
For more detailed examples, please see [here](https://github.com/arc-l/rvg/tree/main/python-example).


## <a name="citation"></a>Citation
If you find this project helpful for your research, please consider citing the following BibTeX entry.
```BibTex
@inproceedings{ZhaYeYu25ICRA,
    author = {Duo Zhang and Zihe Ye and Jingjin Yu},
    title = {Asymptotically-Optimal Multi-Query Path Planning for a Polygonal Robot},
    booktitle={IEEE International Conference on Robotics and Automation},
    year={2025}}
```


