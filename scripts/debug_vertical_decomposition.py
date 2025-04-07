import rvg
import numpy as np
import os
import matplotlib.pyplot as plt

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

if __name__ == "__main__":
    config_file = os.path.join(root, "Configs", "env7.xml")
    robot = rvg.get_robot(config_file)
    boundary = rvg.get_boundary(config_file)
    obstacles = rvg.get_obstacles(config_file)
    start = rvg.get_start(config_file)
    goal = rvg.get_goal(config_file)
    num_threads = 24
    vg = rvg.visibility_graph(robot = robot, 
                              border = boundary, 
                              obstacles = obstacles, 
                              numThreads=num_threads, 
                              resolution=36, 
                              considerSymmetry=True, 
                              hashWithTheta=True, 
                              fineApprox=False, 
                              optimal=True, 
                              verbose=False)
    
    vertical_cells = vg.verticalDecomposition()
    cnt = 0
    for cell in vertical_cells:
        x, y = cell.getX(), cell.getY()
        cx, cy = cell.getCentroid().getCoord()
        plt.plot(x, y, 'b')
        plt.fill(x, y, 'b', alpha=0.5)
        plt.text(cx, cy, str(cnt), fontsize=12, color='white', ha='center', va='center')
        cnt += 1
    plt.savefig("vd.png", dpi=500)
    plt.show()

