#!/usr/bin/env python3
"""Generate static Matplotlib figures for the RVG construction steps.

This is a non-video alternative to scene.py. It saves one PNG per conceptual
step in the RVG explanation.
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-rvg")

import matplotlib.pyplot as plt
import numpy as np
import rvg
from matplotlib.patches import Polygon as PolygonPatch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


COLORS = {
    "obs": "darkcyan",
    "start": "deeppink",
    "goal": "darkviolet",
    "graph_edge": "lightcoral",
    "highlight_edge": "black",
    "graph_vertex": "bisque",
    "semi_vertex": "#EC008C",
    "path": "navy",
    "hole": "tab:red",
    "background": "gainsboro",
    "visible": "#F0AC5F",
}


def polygon_xy(poly):
    return np.column_stack([poly.getX(), poly.getY()])


def vertex_xyz(vertex):
    return np.array([vertex.getX(), vertex.getY(), vertex.getTheta()])


def layer_z(index, layers):
    theta_lb = layers[index].getThetaLb()
    theta_ub = layers[index].getThetaUb()
    return (theta_lb + theta_ub) / 2 if theta_lb < theta_ub else 0


def close_xy(xy):
    if len(xy) == 0:
        return xy
    return np.vstack([xy, xy[0]])


def draw_poly_2d(
    ax,
    poly,
    *,
    facecolor="none",
    edgecolor="black",
    alpha=0.6,
    linewidth=1.5,
    zorder=1,
):
    xy = polygon_xy(poly)
    patch = None
    if facecolor != "none":
        patch = PolygonPatch(
            xy,
            closed=True,
            facecolor=facecolor,
            edgecolor="none",
            alpha=alpha,
            linewidth=0,
            zorder=zorder,
        )
        ax.add_patch(patch)
    closed = close_xy(xy)
    ax.plot(
        closed[:, 0],
        closed[:, 1],
        "-o",
        color=edgecolor,
        markersize=2.0,
        linewidth=linewidth,
        alpha=alpha if facecolor == "none" else 1.0,
        zorder=zorder + 0.1,
    )
    return patch


def draw_poly_3d(
    ax,
    poly,
    z,
    *,
    facecolor="none",
    edgecolor="black",
    alpha=0.5,
    linewidth=1.0,
):
    xy = polygon_xy(poly)
    verts = [[(x, y, z) for x, y in xy]]
    collection = None
    if facecolor != "none":
        collection = Poly3DCollection(
            verts,
            facecolors=facecolor,
            edgecolors="none",
            linewidths=0,
            alpha=alpha,
        )
        ax.add_collection3d(collection)
    closed = close_xy(xy)
    ax.plot(
        closed[:, 0],
        closed[:, 1],
        np.full(len(closed), z),
        "-o",
        color=edgecolor,
        markersize=2.0,
        linewidth=linewidth,
        alpha=1.0 if facecolor != "none" else alpha,
    )
    return collection


def draw_robot_shape_3d(
    ax,
    robot,
    pose,
    *,
    facecolor=None,
    edgecolor=None,
    alpha=0.18,
    linewidth=1.0,
):
    """Draw the configured robot polygon at an SE(2) pose in theta space."""
    facecolor = facecolor or COLORS["path"]
    edgecolor = edgecolor or facecolor
    placed_robot = robot.moveToCopy(pose.getX(), pose.getY(), pose.getTheta())
    return draw_poly_3d(
        ax,
        placed_robot,
        pose.getTheta(),
        facecolor=facecolor,
        edgecolor=edgecolor,
        alpha=alpha,
        linewidth=linewidth,
    )


def draw_robot_shape_2d(
    ax,
    robot,
    pose,
    *,
    facecolor=None,
    edgecolor=None,
    alpha=0.18,
    linewidth=1.0,
    zorder=7,
):
    facecolor = facecolor or COLORS["path"]
    edgecolor = edgecolor or facecolor
    placed_robot = robot.moveToCopy(pose.getX(), pose.getY(), pose.getTheta())
    return draw_poly_2d(
        ax,
        placed_robot,
        facecolor=facecolor,
        edgecolor=edgecolor,
        alpha=alpha,
        linewidth=linewidth,
        zorder=zorder,
    )


def sampled_path_poses(path, *, max_samples=18):
    if len(path) <= max_samples:
        return list(path)
    indices = np.linspace(0, len(path) - 1, max_samples, dtype=int)
    return [path[i] for i in np.unique(indices)]


def draw_robot(ax, pose, *, label=None, color=None, alpha=0.8):
    color = color or COLORS["start"]
    width = 0.5
    height = 1.0
    corners = np.array(
        [
            [-width / 2, -height / 2],
            [width / 2, -height / 2],
            [width / 2, height / 2],
            [-width / 2, height / 2],
        ]
    )
    theta = pose.getTheta()
    rot = np.array(
        [
            [math.cos(theta), -math.sin(theta)],
            [math.sin(theta), math.cos(theta)],
        ]
    )
    xy = corners @ rot.T + np.array([pose.getX(), pose.getY()])
    patch = PolygonPatch(
        xy,
        closed=True,
        facecolor=color,
        edgecolor=color,
        alpha=alpha,
        linewidth=1.5,
        zorder=5,
    )
    ax.add_patch(patch)
    if label:
        ax.text(pose.getX(), pose.getY(), label, ha="center", va="center", zorder=6)
    return patch


def set_2d_axes(ax, title, *, limits=((-2.8, 2.8), (-2.8, 2.8))):
    ax.set_facecolor(COLORS["background"])
    if title:
        ax.set_title(title)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(*limits[0])
    ax.set_ylim(*limits[1])
    ax.set_axis_off()


def set_3d_axes(
    ax,
    title,
    *,
    limits=((-2.8, 2.8), (-2.8, 2.8), (-0.1, math.pi + 0.2)),
    azim=-55,
):
    ax.set_title(title)
    ax.set_xlim(*limits[0])
    ax.set_ylim(*limits[1])
    ax.set_zlim(*limits[2])
    ax.grid(False)
    ax.xaxis.pane.set_alpha(0.0)
    ax.yaxis.pane.set_alpha(0.0)
    ax.zaxis.pane.set_alpha(0.0)
    ax.set_axis_off()
    ax.view_init(elev=24, azim=azim)


def save(fig, output_dir, index, name):
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / f"{index:02d}_{name}.png"
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    print(path)


def draw_base_map(ax, boundary, obstacles, start, goal):
    draw_poly_2d(ax, boundary, facecolor="none", edgecolor=COLORS["obs"], alpha=1.0, linewidth=1.0)
    for obstacle in obstacles:
        draw_poly_2d(
            ax,
            obstacle,
            facecolor=COLORS["obs"],
            edgecolor=COLORS["obs"],
            alpha=0.35,
            linewidth=1.0,
            zorder=3,
        )
    draw_robot(ax, start, color=COLORS["start"], alpha=1.0)
    draw_robot(ax, goal, color=COLORS["goal"], alpha=1.0)


def draw_layer_environment_2d(ax, layer):
    draw_poly_2d(
        ax,
        layer.getShrinkedBorder(),
        facecolor="none",
        edgecolor=COLORS["goal"],
        alpha=0.9,
        linewidth=1.0,
    )
    for grown in layer.getGrownObs():
        draw_poly_2d(
            ax,
            grown,
            facecolor="none",
            edgecolor=COLORS["obs"],
            alpha=1.0,
            linewidth=1.0,
            zorder=3,
        )


def draw_layer_environment_3d(ax, layers, layer_indices):
    for i in layer_indices:
        z = layer_z(i, layers)
        layer = layers[i]
        draw_poly_3d(
            ax,
            layer.getShrinkedBorder(),
            z,
            facecolor=COLORS["goal"],
            edgecolor=COLORS["goal"],
            alpha=0.08,
            linewidth=1.0,
        )
        for grown in layer.getGrownObs():
            draw_poly_3d(
                ax,
                grown,
                z,
                facecolor=COLORS["obs"],
                edgecolor=COLORS["obs"],
                alpha=0.35,
                linewidth=1.0,
            )


def draw_3d_edges_at_vertex_theta(ax, edges, *, color=None, linewidth=1.0, alpha=0.35):
    color = color or COLORS["graph_edge"]
    for v0, v1 in edges:
        ax.plot(
            [v0.getX(), v1.getX()],
            [v0.getY(), v1.getY()],
            [v0.getTheta(), v1.getTheta()],
            color=color,
            linewidth=linewidth,
            alpha=alpha,
        )


def positive_vertices(poly):
    vertices = [(x, y) for x, y in polygon_xy(poly) if x > 0 and y > 0]
    return vertices or [tuple(xy) for xy in polygon_xy(poly)[:2]]


def generate_figures(output_dir, resolution, num_threads):
    config_path = Path(__file__).with_name("manimConfig.xml")
    robot = rvg.get_robot(str(config_path))
    boundary = rvg.get_map(5)
    obstacles = rvg.get_obstacles(str(config_path))
    start = rvg.get_start(str(config_path))
    goal = rvg.get_goal(str(config_path))
    vg = rvg.rvg(
        robot=robot,
        border=boundary,
        obstacles=obstacles,
        resolution=resolution,
        fineApprox=False,
        numThreads=num_threads,
        optimal=False,
        verbose=False,
    )
    layers = vg.getLayers()
    forward_edges = vg.getEdgeLayersForward()
    backward_edges = vg.getEdgeLayersBackward()
    first_layer = layers[0]

    fig, ax = plt.subplots(figsize=(7, 7))
    draw_base_map(ax, boundary, obstacles, start, goal)
    set_2d_axes(ax, None)
    save(fig, output_dir, 1, "map_start_goal")

    fig, ax = plt.subplots(figsize=(7, 7))
    draw_robot(ax, start, color=COLORS["start"], alpha=0.65)
    beta_pose = type(start)()
    beta_pose.setPos(start.getX(), start.getY())
    beta_pose.setTheta(start.getTheta() + math.pi / 4)
    draw_robot(ax, beta_pose, color=COLORS["goal"], alpha=0.55)
    center = np.array([start.getX(), start.getY()])
    for theta in [start.getTheta(), start.getTheta() + math.pi / 4]:
        direction = np.array([math.sin(theta), math.cos(theta)])
        end = center + 0.75 * direction
        ax.plot([center[0], end[0]], [center[1], end[1]], color=COLORS["obs"], linewidth=1.0)
    set_2d_axes(ax, None)
    save(fig, output_dir, 2, "rotation_interval")

    fig, ax = plt.subplots(figsize=(7, 7))
    bbox = first_layer.getRobotBBox().moveToCopy(start.getX(), start.getY(), 0)
    draw_robot(ax, start, color=COLORS["start"], alpha=0.35)
    draw_robot(ax, beta_pose, color=COLORS["goal"], alpha=0.35)
    draw_poly_2d(ax, bbox, facecolor="none", edgecolor=COLORS["obs"], alpha=1.0, linewidth=1.0)
    set_2d_axes(ax, None)
    save(fig, output_dir, 3, "bounding_polygon")

    fig, ax = plt.subplots(figsize=(7, 7))
    draw_poly_2d(ax, boundary, facecolor="none", edgecolor=COLORS["obs"], alpha=1.0, linewidth=1.0)
    for obstacle in obstacles:
        draw_poly_2d(ax, obstacle, facecolor="none", edgecolor=COLORS["obs"], alpha=1.0)
    for grown in first_layer.getGrownObs():
        draw_poly_2d(ax, grown, facecolor="none", edgecolor=COLORS["obs"], alpha=1.0, zorder=4)
    draw_poly_2d(
        ax,
        first_layer.getShrinkedBorder(),
        facecolor="none",
        edgecolor=COLORS["goal"],
        alpha=1.0,
        linewidth=2.5,
        zorder=5,
    )
    set_2d_axes(ax, "Minkowski-grown obstacle and shrunk border")
    save(fig, output_dir, 4, "grown_obstacle_shrunk_border")

    path = vg.shortestPath(start, goal, 10, False)

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    draw_layer_environment_3d(ax, layers, range(len(layers)))
    set_3d_axes(ax, "Rotation space sliced into layers")
    save(fig, output_dir, 5, "all_layers")

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    draw_layer_environment_3d(ax, layers, range(min(2, len(layers))))
    for i in range(min(2, len(layers))):
        z = layer_z(i, layers)
        for grown in layers[i].getGrownObs():
            xy = polygon_xy(grown)
            ax.scatter(xy[:, 0], xy[:, 1], np.full(len(xy), z), color=COLORS["semi_vertex"], s=20)
    set_3d_axes(ax, "First two layers and grown-obstacle vertices")
    save(fig, output_dir, 6, "first_two_layers")

    visible_samples = []
    for layer_index in range(min(2, len(layers))):
        vertices = positive_vertices(layers[layer_index].getGrownObs()[0])
        visible_samples.append((layer_index, vertices[0]))

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    draw_layer_environment_3d(ax, layers, range(min(2, len(layers))))
    for layer_index, (x, y) in visible_samples:
        z = layer_z(layer_index, layers)
        visible = layers[layer_index].getVisibleAreaPolygon(x, y)
        draw_poly_3d(
            ax,
            visible,
            z + 0.025,
            facecolor=COLORS["visible"],
            edgecolor=COLORS["visible"],
            alpha=0.8,
            linewidth=1.0,
        )
        ax.scatter([x], [y], [z + 0.05], color=COLORS["semi_vertex"], s=50)
    set_3d_axes(ax, "Visible areas in the first two layers", azim=35)
    save(fig, output_dir, 7, "visible_area_queries")

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    draw_layer_environment_3d(ax, layers, range(min(2, len(layers))))
    layer0_vertices = positive_vertices(layers[0].getGrownObs()[0])
    layer1_vertices = positive_vertices(layers[1].getGrownObs()[0])
    if len(layer0_vertices) > 1 and len(layer1_vertices) > 1:
        ax0, ay0 = layer0_vertices[1]
        bx, by = layer1_vertices[1]
        z0 = layer_z(0, layers)
        z1 = layer_z(1, layers)
        visible = layers[1].getVisibleAreaPolygon(bx, by)
        draw_poly_3d(
            ax,
            visible,
            z1 + 0.025,
            facecolor=COLORS["visible"],
            edgecolor=COLORS["visible"],
            alpha=0.8,
            linewidth=1.0,
        )
        set_3d_axes(ax, "A rotates to A' before connecting to B", azim=35)
        rotation_line = ax.plot(
            [ax0, ax0],
            [ay0, ay0],
            [z0, z1],
            color=COLORS["highlight_edge"],
            linewidth=3.0,
            zorder=100,
        )
        translation_line = ax.plot(
            [ax0, bx],
            [ay0, by],
            [z1, z1],
            color=COLORS["highlight_edge"],
            linewidth=3.0,
            zorder=100,
        )
        points = ax.scatter(
            [ax0, bx, ax0],
            [ay0, by, ay0],
            [z0, z1, z1],
            color=COLORS["highlight_edge"],
            s=50,
            depthshade=False,
            zorder=101,
        )
        for line in rotation_line + translation_line:
            line.set_zorder(100)
        points.set_zorder(101)
    else:
        set_3d_axes(ax, "A rotates to A' before connecting to B", azim=35)
    save(fig, output_dir, 8, "example_interlayer_edge")

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    draw_layer_environment_3d(ax, layers, range(min(2, len(layers))))
    if len(forward_edges) > 0:
        draw_3d_edges_at_vertex_theta(ax, forward_edges[0], linewidth=1.2)
    set_3d_axes(ax, "Forward propagation from layer 0 to layer 1")
    save(fig, output_dir, 9, "forward_first_pair")

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    draw_layer_environment_3d(ax, layers, range(len(layers)))
    for i in range(len(layers) - 1):
        if i < len(forward_edges):
            draw_3d_edges_at_vertex_theta(ax, forward_edges[i], alpha=0.75)
    set_3d_axes(ax, "Forward vertex propagation across all layers")
    save(fig, output_dir, 10, "forward_propagation")

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    draw_layer_environment_3d(ax, layers, range(len(layers)))
    for i in range(len(layers) - 1, 0, -1):
        if i < len(backward_edges):
            draw_3d_edges_at_vertex_theta(ax, backward_edges[i], alpha=0.75)
    set_3d_axes(ax, "Backward vertex propagation across all layers")
    save(fig, output_dir, 11, "backward_propagation")

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    draw_layer_environment_3d(ax, layers, range(len(layers)))
    for i in range(len(layers) - 1):
        if i < len(forward_edges):
            draw_3d_edges_at_vertex_theta(ax, forward_edges[i], alpha=0.65)
    for i in range(len(layers) - 1, 0, -1):
        if i < len(backward_edges):
            draw_3d_edges_at_vertex_theta(ax, backward_edges[i], color=COLORS["graph_edge"], alpha=0.6)
    set_3d_axes(ax, "Constructed RVG")
    save(fig, output_dir, 12, "constructed_rvg")

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    draw_layer_environment_3d(ax, layers, range(len(layers)))
    ax.scatter([start.getX()], [start.getY()], [start.getTheta()], color=COLORS["start"], s=55, label="start")
    ax.scatter([goal.getX()], [goal.getY()], [goal.getTheta()], color=COLORS["goal"], s=55, label="goal")
    for source, adjacent in [(start, vg.getAdjacentVertices(start)), (goal, vg.getAdjacentVertices(goal))]:
        for vertex in adjacent:
            ax.plot(
                [source.getX(), vertex.getX()],
                [source.getY(), vertex.getY()],
                [source.getTheta(), vertex.getTheta()],
                color=COLORS["graph_edge"],
                linewidth=1.0,
                alpha=0.35,
            )
    ax.legend(loc="upper right")
    set_3d_axes(ax, "Start and goal connected to visible RVG vertices")
    save(fig, output_dir, 13, "start_goal_connections")

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    draw_layer_environment_3d(ax, layers, range(len(layers)))
    path_points = []
    for i, vertex in enumerate(path):
        if i == 0:
            path_points.append(vertex_xyz(start))
        elif i == len(path) - 1:
            path_points.append(vertex_xyz(goal))
        else:
            path_points.append(vertex_xyz(vertex))
    path_points = np.array(path_points)
    if len(path_points) > 0:
        ax.plot(path_points[:, 0], path_points[:, 1], path_points[:, 2], color=COLORS["path"], linewidth=3)
        ax.scatter(path_points[:, 0], path_points[:, 1], path_points[:, 2], color=COLORS["path"], s=18)
        for pose in sampled_path_poses(path):
            draw_robot_shape_3d(
                ax,
                robot,
                pose,
                facecolor=COLORS["path"],
                edgecolor=COLORS["path"],
                alpha=0.18,
                linewidth=0.8,
            )
        draw_robot_shape_3d(
            ax,
            robot,
            start,
            facecolor=COLORS["start"],
            edgecolor=COLORS["start"],
            alpha=0.45,
            linewidth=1.2,
        )
        draw_robot_shape_3d(
            ax,
            robot,
            goal,
            facecolor=COLORS["goal"],
            edgecolor=COLORS["goal"],
            alpha=0.45,
            linewidth=1.2,
        )
    set_3d_axes(ax, "Shortest path through the RVG")
    save(fig, output_dir, 14, "final_solution_path_3d")

    path_2d = vg.shortestPath(start, goal, 10, True)
    fig, ax = plt.subplots(figsize=(7, 7))
    draw_base_map(ax, boundary, obstacles, start, goal)
    if len(path_2d) > 0:
        xs = [v.getX() for v in path_2d]
        ys = [v.getY() for v in path_2d]
        for pose in sampled_path_poses(path_2d):
            draw_robot_shape_2d(
                ax,
                robot,
                pose,
                facecolor=COLORS["path"],
                edgecolor=COLORS["path"],
                alpha=0.16,
                linewidth=0.8,
                zorder=7,
            )
        ax.plot(xs, ys, color=COLORS["path"], linewidth=3, zorder=8)
        ax.scatter(xs, ys, color=COLORS["path"], s=14, zorder=9)
        draw_robot_shape_2d(
            ax,
            robot,
            start,
            facecolor=COLORS["start"],
            edgecolor=COLORS["start"],
            alpha=0.5,
            linewidth=1.2,
            zorder=10,
        )
        draw_robot_shape_2d(
            ax,
            robot,
            goal,
            facecolor=COLORS["goal"],
            edgecolor=COLORS["goal"],
            alpha=0.5,
            linewidth=1.2,
            zorder=10,
        )
    set_2d_axes(ax, "Final 2D solution path")
    save(fig, output_dir, 15, "final_solution_path")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        default=str(Path(__file__).with_name("matplotlib_steps")),
        help="Directory where PNG step figures are written.",
    )
    parser.add_argument("--resolution", type=int, default=8, help="RVG angular resolution.")
    parser.add_argument("--num-threads", type=int, default=1, help="RVG build thread count.")
    args = parser.parse_args()
    generate_figures(Path(args.output_dir), args.resolution, args.num_threads)


if __name__ == "__main__":
    main()
