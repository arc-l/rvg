## Asymptotically-Optimal Multi-Query Path Planning for Moving A Convex Polygon in 2D [![YouTube](https://badges.aleen42.com/src/youtube.svg)](https://youtu.be/iH7PsPNSOAY) [![Static Badge](https://img.shields.io/badge/RVG-arXiv-blue )](https://arxiv.org/pdf/2409.03920)


<a href="https://duozhangrobotics.github.io/" target="_blank"><nobr>Duo Zhang</nobr></a> &emsp;
<nobr>Zihe Ye</nobr> &emsp;
<a href="https://arc-l.github.io/group.html" target="_blank"><nobr>Jingjin Yu</nobr></a> &emsp; (Accepted to [ICRA 2025](https://2025.ieee-icra.org/))

<div align="center">
<p float="left">
  <img src="https://github.com/user-attachments/assets/ad62b570-e4dd-4854-ab40-2bdfb43a1818" width="30%" />
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
- [Citation](#citation)

## <a name="intro"></a>Introduction
### <a name="rvg"></a>What does a rotation-stacked visibility graph (RVG) do?  
RVG constructs a layered visibility graph over multiple discrete orientations, allowing efficient, high-quality path planning for polygonal robots with both translation and rotation in 2D environments. It maintains completeness and asymptotic optimality while enabling rapid queries across diverse planning scenarios. The video below (with narrations) illustrates how RVG works. 

https://github.com/user-attachments/assets/e8292aab-cb14-4794-8171-dd2f7ae271db

### <a name="compare"></a>Comparisons with SOTA SE(2) Rigid Body Planners
RVG, as a multi-query method dedicated to path planning for rigid bodies in SE(2), outperforms sampling-based planners in such settings, producing shorter paths with significantly lower planning times. Below, we observe that RVG does better in efficiency and optimality for ten typical problem instances similar to the example on the right above.
![comparison_together](https://github.com/user-attachments/assets/7d2d459e-5ced-488b-a054-98cae8f0f26c)

Given RVG’s time budget, sampling-based methods often produce longer paths, with cost ratios exceeding 1.0. This further corroborates RVG’s superior solution quality under the same amount of compute.
    <div align="center">
    <p float="left">
    <img src=
      "https://github.com/user-attachments/assets/09e63f81-a89a-4f2a-a1b2-e97df4f5c867"
      width="90%" />
    </p>
  </div>
  
### <a name="examples"></a>Additional Examples
RVG works with rigid bodies that are non-convex: 
  <div align="center">
  <p float="left">
    <img src=
      "https://github.com/user-attachments/assets/567d7675-561f-4a3d-ae1c-d47713370128"
      width="45%" />
    <img src=
      "https://github.com/user-attachments/assets/1225cd51-1996-4cf4-953c-3e00bbb623b8"
      width="45%" />
  </p>
  </div>
RVG can produce different solutions based on the relative weighting of translation and rotation during the search process.
  <div align="center">
  <p float="left">
    <img src=
      "https://github.com/user-attachments/assets/040a7b34-5e9e-4d0c-aed4-2a7dd7cee587"
      width="45%" />
    <img src=
      "https://github.com/user-attachments/assets/190e84ec-8a3e-4f83-ac2c-be2240138627"
      width="45%" />
    <img src=
      "https://github.com/user-attachments/assets/ab5e146e-ae8d-424e-ba43-2a6d173356f5"
      width="45%" />
      <img src=
      "https://github.com/user-attachments/assets/bf9f0740-16fc-490c-ab49-afe5199717b3"
      width="45%" />
  </p>
  </div>

## <a name="install"></a>Installation and Example Usage

We provide C++ implementation of RVG with python bindings. Give it a try!

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
    To have a clean python environment, create a virtual environment and activate
    ```bash
    conda create -n rvg python==3.10 numpy matplotlib # matplotlib is for visualizing the solutions and environments.
    conda install -c conda-forge cpython # for different versions of libpython
    conda activate rvg
    ```
    Then you can simply run `cmake` without setting the `PYTHON_EXECUTABLE` because cmake will use the python in the conda environment by default.


### To build python package
```bash
# activate the environment you want
pip install .
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
from rvg import vertex, polygon, rvg 
```
Then you can create a vertex given its `x` and `y`
```python
v = vertex(x=0, y=0)
```
Since the vertex in RVG is in the SE(2) space, you can specify its rotation `theta` and the rotation range `[theta_lb, theta_ub]`. 
```python
v = vertex(x=0, y=0, theta_lb=0, theta_ub=2*np.pi, theta=0)
```
You can change everything field if you want:
```python
v.setTheta(theta)
v.setBounds(theta_lb, theta_ub)
v.setPos(x, y)
```
You can also create a polygon given a list of vertices:
```python
p = polygon(vertices=[
      vertex(0, 0),
      vertex(2, 0),
      vertex(2, 2),
      vertex(0, 2)
    ], sortVertices=False)
```
If you have random vertices, you can set `sortVertices` to `True` to sort them in the counterclockwise order. If your vertices are already sorted, you can ignore `sortVertices` - its default value is False.
If you want to set a specific rotation center rather its default centroid, especially when the robot is non-convex, you can use another constructor:
```python
p = polygon(vertices=vertices, center=center, sortVertices=False)
```
Then you can construct a RVG:
```python
vg = rvg(robot=robot, # represented by a polygon
         border = border, # represented by a polygon
         obstacles = obstacles, # represented by a list of polygons 
         resolution=18, 
         numThreads=1,  # number of cores for parallelization
         verbose=False, # show running details
         fineApprox=True # Use a finer approximation of the rotation range
         )
```
and then set the weights on translation and rotation. The default value is 1.0 for translation and 0.0 for rotation.
```python
vg.setWeight(euclideanWeight=1.0, rotationalWeight=0.1)
```
Finally, you can find the shortest path from `start(vertex)` to `goal(vertex)`
```python
path = vg.shortestPath(start=start, goal=goal, interpolationDensity=10) 
```
The `interpolationDensity` is used to linearly interpolate the solution, the default value is `0`.
For more detailed examples, please see [here](https://github.com/arc-l/rvg/tree/main/python-example).

### Configuration files
Except setting up everything in your code, you can also set up the environment in a configuration file. Here's an example of the configuration file:
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<root>
  <environment>
    <robot>
      <Polygon isObs="false">
        <Vertex x="-3" y="0"/>
        <Vertex x="-2" y="0"/>
        <Vertex x="-2" y="2"/>
        <Vertex x="-3" y="2"/>
      </Polygon>
    </robot>
    <useBoundary>1</useBoundary>
    <boundary>
      <Polygon isObs="true">
        <Vertex x="-5" y="-0.2"/>
        <Vertex x="10" y="-0.2"/>
        <Vertex x="10" y="8"/>
        <Vertex x="-5" y="8"/>
      </Polygon>
    </boundary>
    <obstacles>
      <Polygon isObs="true">
        <Vertex x="2" y="1"/>
        <Vertex x="4" y="1"/>
        <Vertex x="4" y="5"/>
        <Vertex x="2" y="5"/>
      </Polygon>
    </obstacles>
    <mapSize>45</mapSize>
  </environment>

  <plannerSettings>
    <start>
      <Vertex x="-2.5" y="1.2" theta="0" thetaLb="0" thetaUb="6.28"/>
    </start>
    <goal>
      <Vertex x="8" y="1.2" theta="0.78" thetaLb="0" thetaUb="6.28"/>
    </goal>
  </plannerSettings>
</root>
```
#### Breakdown
| Tag              | Description |
|------------------|-------------|
| `<robot>`        | Polygon defining the robot's shape. |
| `<useBoundary>`  | Set to 1 to use the custom boundary. |
| `<boundary>`     | World boundary polygon. |
| `<obstacles>`    | List of obstacle polygons. |
| `<mapSize>`      | Used when `useBoundary=0` to create a default square map. |
| `<start>`        | Start state `Vertex`, including x, y, θ, and angular bounds. |
| `<goal>`         | Goal state `Vertex`, including x, y, θ, and angular bounds. |

After you have the configuration file and it's `path`, you can read everything from it by:
```python
robot = rvg.get_robot(path)
boundary = rvg.get_boundary(path)
obstacles = rvg.get_obstacles(path)
start = rvg.get_start(path)
goal = rvg.get_goal(path)
```
Feel free to add more fields into the configuration file and use them!


## <a name="citation"></a>Citation
If you find this project helpful for your research, please consider citing the following BibTeX entry.
```BibTex
@inproceedings{ZhaYeYu25ICRA,
    author = {Duo Zhang and Zihe Ye and Jingjin Yu},
    title = {Asymptotically-Optimal Multi-Query Path Planning for a Polygonal Robot},
    booktitle={IEEE International Conference on Robotics and Automation},
    year={2025}}
```


