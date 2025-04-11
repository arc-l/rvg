import rvg
import os
import matplotlib.pyplot as plt

if __name__ == "__main__":
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = os.path.join(root, "Configs", "env3.xml") 
    robot = rvg.get_robot(path)
    boundary = rvg.get_boundary(path)
    obstacles = rvg.get_obstacles(path)
    start = rvg.get_start(path)
    goal = rvg.get_goal(path)

    total_time = []
    for num_threads in range(1, 24):
        vg = rvg.visibility_graph(robot=robot,
                                    border=boundary,
                                    obstacles=obstacles,
                                    resolution=36,
                                    numThreads=num_threads,
                                    verbose=False,
                                    fineApprox=False,
                                    optimal=True,
                                    considerSymmetry=True
        )
        path = vg.shortestPath(start, goal)
        total_time.append(vg.getTotalTime())
        print(f"Total time for {num_threads} threads: {vg.getTotalTime()}, path length: {vg.getPathLength()}")

    plt.plot(total_time)
    plt.xlabel('Number of threads')
    plt.ylabel('Total time (s)')
    plt.title('Total time vs number of threads')
    plt.savefig(os.path.join(root, "Results/time_vs_cores.png"), dpi=500)
    plt.show()

