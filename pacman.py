import rvg

path = "Configs/pacman.xml"
robot = rvg.get_robot(path)
boundary = rvg.get_boundary(path)
obstacles = rvg.get_obstacles(path)
start = rvg.get_start(path)
goal = rvg.get_goal(path)
print(f"start = {start}")
print(f"goal= {goal}")
vg = rvg.rvg(robot=robot, # represented by a polygon
         border = boundary, # represented by a polygon
         obstacles = obstacles, # represented by a list of polygons 
         resolution=36, 
         numThreads=24,  # number of cores for parallelization
         verbose=False, # show running details
         fineApprox=True # Use a finer approximation of the rotation range
         )

vg.setWeight(euclideanWeight=1.0, rotationalWeight=0.1)
path = vg.shortestPath(start=start, goal=goal, interpolationDensity=10, unwrap=True) 
