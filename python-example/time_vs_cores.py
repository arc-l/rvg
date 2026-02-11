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

    max_threads = os.cpu_count() or 1
    requested_max_threads = 23
    max_threads_to_test = min(requested_max_threads, max_threads)

    total_time = []
    thread_counts = list(range(1, max_threads_to_test + 1))
    for num_threads in thread_counts:
        vg = rvg.rvg(robot=robot,
                    border=boundary,
                    obstacles=obstacles,
                    resolution=36,
                    numThreads=num_threads,
                    verbose=False,
                    fineApprox=False,
                    optimal=True
        )
        path = vg.shortestPath(start, goal)
        total_time.append(vg.getTotalTime())
        print(f"Total time for {num_threads} threads: {vg.getTotalTime()}, path length: {vg.getPathLength()}")

    plt.plot(thread_counts, total_time)
    plt.xlabel('Number of threads')
    plt.ylabel('Total time (s)')
    plt.title('Total time vs number of threads')
    plt.savefig(os.path.join(root, "Results/time_vs_cores.png"), dpi=500)
    plt.show()
