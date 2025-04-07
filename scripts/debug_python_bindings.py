from rvg import vertex, polygon, visibility_graph
import matplotlib.pyplot as plt
import numpy as np
import os


def generate_vertices():
    vertices_upper = [
        vertex(0, 0),
        vertex(5, 0),
        vertex(5, 5),
        vertex(0, 5)
    ]

    vertices_lower = [
        vertex(0, -5),
        vertex(5, -5),
        vertex(5, -1),
        vertex(0, -1)
    ]

    border = [
        vertex(-30, -6),
        vertex(30, -6),
        vertex(30, 6),
        vertex(-30, 6),
    ]

    robot = [
        vertex(0, 0),
        vertex(2, 0),
        vertex(2, 3),
        vertex(0, 3)
    ]

    return vertices_upper, vertices_lower, border, robot


if __name__ == "__main__":
    vertices_upper, vertices_lower, border, robot = generate_vertices()
    print(vertices_upper)
    print(vertices_lower)
    polygon_upper = polygon(vertices_upper, False)
    polygon_lower = polygon(vertices_lower, False)
    border = polygon(border, False)
    robot = polygon(robot, False)
    print("Polygon_upper: ", polygon_upper)
    print("Polygon_upper's vertices: ", polygon_upper.getVertices())
    print("Polygon_lower: ", polygon_lower)
    print("Polygon_lower's vertices: ", polygon_lower.getVertices())
    print("Border: ", border)
    print("Border's vertices: ", border.getVertices())

    obstacles = [polygon_upper, polygon_lower]


    start = vertex(-10, 0, 0, 2 * np.pi, 0, 2 * np.pi, True)
    goal = vertex(20, 0, 0, 2 * np.pi, 0, 2 * np.pi, True)

    plt.plot(polygon_upper.getX(), polygon_upper.getY(), 'r-o')
    plt.plot(polygon_lower.getX(), polygon_lower.getY(), 'r-o')
    plt.plot(border.getX(), border.getY(), 'g-o')
    robot.moveTo(goal.getX(), goal.getY(), goal.getTheta())
    plt.plot(robot.getX(), robot.getY(), 'b-o')
    robot.moveTo(start.getX(), start.getY(), start.getTheta())
    plt.plot(robot.getX(), robot.getY(), 'b-o')
    plt.show()

    vg = visibility_graph(robot=robot, border = border, obstacles = obstacles, resolution=18, considerSymmetry=True, hashWithTheta=True, numThreads=1, verbose=False, fineApprox=True)
    path = vg.shortestPath(start, goal)
    vg.draw("test.png", True, True, True, True)
    print("path = ", path)
    os.remove("test.png")
