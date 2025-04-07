import pyswarms as ps
import numpy as np
from shapely.geometry import Point, Polygon
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt


def f(points: np.ndarray, capsule: Polygon) -> float:
    # create the convex hull with the points
    npoints = points.reshape(-1, 2)
    convex_hull = ConvexHull(npoints)
    convex_hull = Polygon(npoints[convex_hull.vertices])
    res = - convex_hull.area
    # Check if the convex hull is inside the capsule
    if not capsule.contains(convex_hull):
        res = 10000 
    return res



def create_capsule():
    # Parameters
    length = 2  # Total length of the capsule
    width = 1    # Width (diameter) of the capsule

    # Calculations
    radius = width / 2.0
    rectangle_length = length - 2 * radius

    # Create central rectangle
    rectangle = Polygon([
        (-rectangle_length / 2, -radius),
        (rectangle_length / 2, -radius),
        (rectangle_length / 2, radius),
        (-rectangle_length / 2, radius)
    ])

    # Create semicircles
    semicircle_left = Point(-rectangle_length / 2, 0).buffer(radius, resolution=50)
    semicircle_right = Point(rectangle_length / 2, 0).buffer(radius, resolution=50)

    # Combine shapes to form capsule
    capsule = rectangle.union(semicircle_left).union(semicircle_right)
    return capsule

# Plotting
# fig, ax = plt.subplots()
# x, y = capsule.exterior.xy
# ax.plot(x, y, label='Capsule')
# ax.set_aspect('equal', 'box')
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.legend()
# plt.show()



if __name__ == '__main__':
    options = {'c1': 0.5, 'c2': 0.3, 'w': 0.9}

    # Define the number of dimensions and particles
    dimensions = 8  # Adjust based on the problem
    n_particles = 10000
    x_max = 2 * np.ones(dimensions)
    x_min = -2 * np.ones(dimensions)
    bounds = (x_min, x_max)

    # Initialize the optimizer
    optimizer = ps.single.GlobalBestPSO(
                                        n_particles=n_particles, 
                                        dimensions=dimensions, 
                                        options=options,
                                        bounds=bounds
                                        )
    capsule = create_capsule()
    kwargs = {'capsule': capsule}

    # Optimize the objective function
    best_cost, best_pos = optimizer.optimize(f, iters=10000, **kwargs)
    print(f'Best cost: {best_cost}\nBest position: {best_pos}')

    # Plotting
    fig, ax = plt.subplots()
    x, y = capsule.exterior.xy
    ax.plot(x, y, label='Capsule')
    ax.set_aspect('equal')

    # Create the convex hull
    npoints = best_pos.reshape(-1, 2)
    convex_hull = ConvexHull(npoints)
    convex_hull = Polygon(npoints[convex_hull.vertices])
    x, y = convex_hull.exterior.xy
    ax.plot(x, y, label='Convex Hull')
    ax.plot(npoints[:, 0], npoints[:, 1], 'o', label='Points')
    plt.show()

