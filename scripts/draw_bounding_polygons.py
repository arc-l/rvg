import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, Polygon, Arc
import rvg
import math
import numpy as np
import os

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def line_intersection(p1, p2, p3, p4):
    """
    Calculate the intersection point of two lines defined by points p1, p2 and p3, p4.
    Each point is a tuple (x, y).
    Returns a tuple (x, y) of the intersection point or None if the lines are parallel.
    """
    # Unpack points
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4

    # Calculate the denominators
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if denom == 0:
        return None  # Lines are parallel

    # Calculate the numerators
    t_num = (x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)
    u_num = (x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2)

    # Calculate the parameters
    t = t_num / denom
    u = -u_num / denom

    # Calculate the intersection point
    intersection_x = x1 + t * (x2 - x1)
    intersection_y = y1 + t * (y2 - y1)

    return (intersection_x, intersection_y)

def calculate_angle(center, point):
    delta = np.array(point) - np.array(center)
    angle = np.degrees(np.arctan2(delta[1], delta[0]))
    return angle if angle >= 0 else angle + 360


vertices = [
    rvg.vertex(-1.5, -2),
    rvg.vertex(1.5, -2),
    rvg.vertex(1.5, 2),
    rvg.vertex(-1.5, 2)
    ]


polygon = rvg.polygon(vertices, False)

center = (0,0)
radius = 2.5
# 
# Create a rectangle with lower-left corner at (0.2, 0.2), width 0.4, and height 0.3
plt_vertices = [(v.getX(), v.getY()) for v in polygon.getVertices()]

# Add the shapes to the axis
# for resolution in [8, 18, 36, 72, 90, 180, 360]:
for resolution in [8]:
    angle = - math.pi/resolution
    # Create a new figure and axis
    fig, ax = plt.subplots()
    rectangle = Polygon(plt_vertices, edgecolor='black', facecolor='none', linewidth=1)
    ax.add_patch(rectangle)
    polygon_new = polygon.rotateCopy(angle)
    plt_vertices_new = [(v.getX(), v.getY()) for v in polygon_new.getVertices()]
    intersections = []
    for i in range(len(plt_vertices)):
        p1 = plt_vertices[i]
        p2 = plt_vertices_new[(i + 1) % len(plt_vertices)]
        p3 = plt_vertices[(i + 1) % len(plt_vertices)]
        p4 = plt_vertices_new[(i + 2) % len(plt_vertices_new)]
        intersection = line_intersection(p1, p2, p3, p4)
        intersections.append(intersection)

    for i in range(len(intersections)):
        intersection1 = intersections[i]
        intersection2 = intersections[(i + 1) % len(intersections)]
        ax.plot([intersection1[0], intersection2[0]], [intersection1[1], intersection2[1]], color='#39b54a')
    

    # real rotation ranges
    for i in range(len(plt_vertices)):
        # Define the start and end points
        start_point = plt_vertices_new[i]  # Point on the circumference
        end_point = plt_vertices[i]    # Another point on the circumference

        # Calculate the start and end angles
        start_angle = calculate_angle(center, start_point)
        end_angle = calculate_angle(center, end_point)
        arc = Arc(center, 2*radius, 2*radius, angle=0, theta1=start_angle, theta2=end_angle, edgecolor='#39b54a', linewidth=1, linestyle='--')
        ax.add_patch(arc)


    
    rectangle = Polygon(plt_vertices_new, edgecolor='black', facecolor='none', linewidth=1)
    ax.add_patch(rectangle)
    ax.axis('equal')
    # plt.savefig(os.path.join(root, f"Results/overestimation{resolution}.pdf"))
    plt.close(fig)

