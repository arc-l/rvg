import rvg
import numpy as np
import os
import plotly.graph_objects as go

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

if __name__ == "__main__":
    # n = 6
    # for i in range(n):
    #     # x, y = np.cos(2*np.pi*i/n) + 0.001 * np.random.randn(), np.sin(2*np.pi*i/n) + 0.001 * np.random.randn()
    #     x, y = (1 + i/n) * np.cos(2*np.pi*i/n), (1 + i/n) * np.sin(2*np.pi*i/n)
    #     print(f'<Vertex x="{x}" y="{y}"/>')
    # exit()
    config_file = os.path.join(root, "Configs", "manimConfig2.xml")
    robot = rvg.get_robot(config_file)
    boundary = rvg.get_boundary(config_file)
    obstacles = rvg.get_obstacles(config_file)
    start = rvg.get_start(config_file)
    goal = rvg.get_goal(config_file)
    # start = rvg.vertex(x=1.324905, y=2.277335, theta=2.286381, theta_lb=0, theta_ub=2*np.pi, roundUpTheta=np.pi, hashWithTheta=True)
    # goal = rvg.vertex(x=1.729068, y=-0.5058769, theta=3.473205, theta_lb=0, theta_ub=2*np.pi, roundUpTheta=np.pi, hashWithTheta=True)
    num_threads = 24
    vg = rvg.rvg(robot = robot, 
                              border = boundary, 
                              obstacles = obstacles, 
                              numThreads=num_threads, 
                              resolution=180, 
                              fineApprox=False, 
                              optimal=True, 
                              verbose=False)
    
    # vg.setWeight(0.00001, 100000000)
    # vg.setWeight(0, 1)
    path = vg.shortestPath(start, goal, unwrap=True, interpolationDensity=0)
    # vg.animation(os.path.join(root, "Results", "test.gif"), False)
    print("path length=", vg.getPathLength())

    path = vg.shortestPath(start, goal, unwrap=False, interpolationDensity=0)
    for point in path:
        print(point)

    layers = vg.getLayers()
    graph_vertices = vg.getGraphVertices()

    fig = go.Figure()

    # draw the graph vertices
    graph_vertices_x = [point.getX() for point in graph_vertices]
    graph_vertices_y = [point.getY() for point in graph_vertices]
    graph_vertices_z = [point.getTheta() for point in graph_vertices]
    # fig.add_trace(go.Scatter3d(x=graph_vertices_x, y=graph_vertices_y, z=graph_vertices_z, mode='markers', marker=dict(size=2, color='orange')))

    resolution = len(layers)
    for i in range(len(layers)):
        layer = layers[i]
        layer_theta = (layer.getThetaLb() + layer.getThetaUb()) / 2
        # # draw the boundary
        # boundary_x, boundary_y = boundary.getX(), boundary.getY()
        # boundary_z = np.full_like(boundary_x, layer_theta) 
        # fig.add_trace(go.Scatter3d(x=boundary_x, y=boundary_y, z=boundary_z, mode='lines', line=dict(color='darkcyan', width=1)))
        # # draw the obstacles
        # for obstacle in obstacles:
        #     obstacle_x, obstacle_y = obstacle.getX(), obstacle.getY()
        #     obstacle_z = np.full_like(obstacle_x, layer_theta)
        #     fig.add_trace(go.Scatter3d(x=obstacle_x, y=obstacle_y, z=obstacle_z, mode='lines', marker=dict(size=4, color='darkcyan')))
        
        # draw the grown obstacles
        grown_obstacles = layer.getGrownObs()
        for grown_obstacle in grown_obstacles:
            grown_obstacle_x, grown_obstacle_y = grown_obstacle.getX(), grown_obstacle.getY()
            grown_obstacle_z = np.full_like(grown_obstacle_x, layer_theta)
            fig.add_trace(go.Scatter3d(x=grown_obstacle_x, y=grown_obstacle_y, z=grown_obstacle_z, mode='lines+markers', line=dict(color="darkcyan", width=4), marker=dict(size=4, color='#EC008C')))
        
        # draw the holes
        if layer.hasHoles():
            holes = layer.getHoles()
            for hole in holes:
                hole_x, hole_y = hole.getX(), hole.getY()
                hole_z = np.full_like(hole_x, layer_theta)
                fig.add_trace(go.Scatter3d(x=hole_x, y=hole_y, z=hole_z, mode='lines+markers', line=dict(color="darkcyan", width=4), marker=dict(size=4, color='#EC008C')))
        
        # draw the shrunk border
        shrunk_border = layer.getShrinkedBorder()
        shrunk_border_x, shrunk_border_y = shrunk_border.getX(), shrunk_border.getY()
        shrunk_border_z = np.full_like(shrunk_border_x, layer_theta)
        # fig.add_trace(go.Scatter3d(x=shrunk_border_x, y=shrunk_border_y, z=shrunk_border_z, mode='lines+markers', line=dict(color="darkcyan", width=4), marker=dict(size=4, color='#EC008C')))

    # draw the path
    path_x = [point.getX() for point in path]
    path_y = [point.getY() for point in path]
    path_z = [point.getTheta() for point in path]
    fig.update_layout(
        xaxis=dict(scaleanchor="y", scaleratio=1),
        yaxis=dict(scaleanchor="x", scaleratio=1)
    )


    fig.add_trace(go.Scatter3d(x=path_x, y=path_y, z=path_z, mode='lines+markers', line=dict(color="navy", width=4), marker=dict(size=4, color='navy')))
    fig.add_trace(go.Scatter3d(x=[start.getX(), goal.getX()], y=[start.getY(), goal.getY()], z=[start.getTheta(), goal.getTheta()], mode='lines+markers', line=dict(color="brown", width=4), marker=dict(size=4, color='brown')))
    fig.add_trace(go.Scatter3d(x=[start.getX()], y=[start.getY()], z=[start.getTheta()], mode='markers', marker=dict(size=5, color='blue')))
    fig.add_trace(go.Scatter3d(x=[goal.getX()], y=[goal.getY()], z=[goal.getTheta()], mode='markers', marker=dict(size=5, color='red')))

    fig.write_html(os.path.join(root, "Results", "3d_graph1.html"))
    fig.show()
