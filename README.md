# RotationalVisibilityGraph

## Clone this repository with submodules
```bash
git clone --recurse-submodules https://github.com/arc-l/rvg.git 
```

If you have already cloned the repository, you can run the following command to get the submodules
```bash
git submodule update --init --recursive
```

## Requirements
For Ubuntu
```bash
sudo apt install libboost-all-dev  libgmp-dev  libmpfr-dev libtinyxml2-dev qtbase5-dev
```
For Mac brew
```bash
brew install eigen boost tinyxml2
```

## To build c++ library
```bash
mkdir build
cd build
cmake -DPYTHON_EXECUTABLE=$(which python) ..
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

