from manim import *
from manim.scene.three_d_scene import ThreeDScene
from manim_voiceover import VoiceoverScene
from manim_voiceover.services.gtts import GTTSService
import rvg
import os

def getVertices(rvg_polygon, z=0):
    xs = rvg_polygon.getX()
    ys = rvg_polygon.getY()
    padding = [z]*len(xs)
    return zip(xs, ys, padding)

def convert_rvgPolygon_to_manimPolygon(rvg_polygon, opacity=0.8, color=BLUE, stroke_color=WHITE, stroke_width=1, z=0):
    polygon = Polygon(*getVertices(rvg_polygon, z=z), color=color, fill_opacity=opacity)
    if stroke_color and stroke_width:
        polygon.set_stroke(stroke_color, stroke_width)
    return polygon

class RVG(VoiceoverScene, ThreeDScene):
    def construct(self):
        # rvg setup
        path = os.path.join(os.path.dirname(__file__), "manimConfig.xml")
        rvg_robot = rvg.get_robot(path)
        rvg_boundary = rvg.get_map(5)
        rvg_obstacles = rvg.get_obstacles(path)
        rvg_start = rvg.get_start(path)
        rvg_goal = rvg.get_goal(path)
        vg = rvg.visibility_graph(
            robot=rvg_robot, 
            border=rvg_boundary, 
            obstacles=rvg_obstacles, 
            resolution=8, 
            considerSymmetry=True, 
            hashWithTheta=True, 
            simplifiedGeometry=False, 
            fineApprox=False, 
            numThreads=1, 
            incremental=False, 
            optimal=False, 
            verbose=False
            )
        sol = vg.shortestPath(rvg_start, rvg_goal, 0)
        layers = vg.getLayers()
        layerEdgesForward = vg.getEdgeLayersForward()
        layerEdgesBackward= vg.getEdgeLayersBackward()
        for sol_i in sol:
            print(sol_i)
        sol_iter = iter(sol)
        def sol_updater(robot):
            sol_i = next(sol_iter, None)
            if sol_i is not None:
                robot.move_to([sol_i.getX(), sol_i.getY(), 0])
                robot.rotate(sol_i.getTheta())


        # manim setup
        self.set_speech_service(GTTSService())
        map = Square(5)
        map.set_fill(GREY_E, opacity=0.8)
        obstacle = Square(1.5)
        obstacle.set_fill(GREEN, opacity=0.8)
        obstacle.set_stroke(WHITE, 1)
        start = Rectangle(height = 1, width=0.5)
        start.set_fill(PINK, opacity=0.8)
        start.set_stroke(WHITE, 1)
        start.rotate(rvg_start.getTheta())
        start.move_to([rvg_start.getX(), rvg_start.getY(), 0])

        goal = Rectangle(height = 1, width=0.5)
        goal.set_fill(PINK, opacity=0.8)
        goal.set_stroke(WHITE, 1)
        goal.rotate(rvg_goal.getTheta())
        goal.move_to([rvg_goal.getX(), rvg_goal.getY(), 0])
        self.add(map, obstacle, start, goal)

        with self.voiceover(text="This is a map with an green obstacle") as tracker:
            self.play(Create(map),
                    Create(obstacle), 
                    Create(start), 
                    Create(goal),
                    run_time = 1
                    )  # show the circle on screen
            self.play(Wiggle(obstacle), run_time=tracker.duration-1)
        with self.voiceover(text="where a robot is trying to find a path from the start") as tracker:
            self.wait(1)
            self.play(Wiggle(start), run_time=tracker.duration-1) 
        with self.voiceover(text="to the goal.") as tracker:
            self.play(Wiggle(goal), run_time=tracker.duration) 

        # self.next_section(skip_animations=True)
        # robot = Rectangle(height = 1, width=0.5)
        # robot.set_fill(PINK, opacity=0.8)
        # robot.set_stroke(WHITE, 1)
        # goal.rotate(rvg_goal.getTheta())
        # robot.rotate(rvg_start.getTheta())
        # robot.move_to([rvg_start.getX(), rvg_start.getY(), 0])
        # self.add(robot)
        # with self.voiceover(text="This is a path find by RVG.") as tracker:
        #     for i in range(len(sol)):
        #         sol_i = sol[i]
        #         if i == 0:
        #             angle = 0
        #         else:
        #             angle = sol_i.getTheta() -  sol[i-1].getTheta()
        #         self.play(robot.animate.move_to([sol_i.getX(), sol_i.getY(), 0]).rotate(angle), run_time=0.1)

        self.next_section("Building bounding polygons")
        with self.voiceover(text="To incoporate certain range of rotation into the Visibility Graphs.") as tracker:
            ani_time = 1.5
            self.move_camera(
                frame_center=start.get_center(),
                zoom=3.5,
                added_anims=[
                FadeOut(obstacle),
                FadeOut(map),
                FadeOut(goal)
                ],
                run_time=ani_time
            )
            if tracker.duration < ani_time + 1:
                self.wait(ani_time + 1 - tracker.duration)
        
        with self.voiceover(text="For example, from alpha to beta.") as tracker:
            font_size = 20
            alpha = Tex(r"$\alpha$", font_size=font_size)
            beta = Tex(r"$\beta$", font_size=font_size)
            alpha.move_to(start.get_center() + 0.65*UP)
            beta.move_to(start.get_center() + 0.65*UP)
            beta.rotate(PI/4, about_point=start.get_center())
            beta.rotate(-PI/4)
            start1 = start.copy() 
            line1 = Line(start.get_center(), start.get_center() + 0.52*UP)
            line1.set_stroke(WHITE, 2)
            self.add(line1)
            line2 = line1.copy()
            alpha1 = alpha.copy()
            self.play(
                Write(alpha),
                Rotate(alpha1, PI/4, about_point=start.get_center()),
                Transform(alpha1, beta),
                start1.animate.rotate(PI/4),
                Rotate(line2, PI/4, about_point=start.get_center()),
                run_time=1
            )
        
        with self.voiceover(text="We can build a bounding polygon to estimate the range of rotation.") as tracker:
            layer = layers[0]
            bounding_polygon = layer.getRobotBBox()
            polygon = convert_rvgPolygon_to_manimPolygon(bounding_polygon) 
            polygon.move_to(start.get_center())
            polygon1 = polygon.copy()
            polygon1.set_fill(BLUE, opacity=1)
            self.play(
                FadeOut(alpha),
                FadeOut(alpha1),
                FadeOut(line1),
                FadeOut(line2),
                Create(polygon),
                run_time=1
            )
            self.wait(1)
            self.play(
                FadeOut(start),
                FadeOut(start1),
                FadeTransform(polygon, polygon1),
            )
            self.remove(alpha, alpha1, beta, line1, line2, start1, polygon)
            self.add(polygon1)

        with self.voiceover(text="Now we can find the Minkowski sum between the bounding polygon and the obstacle") as tracker:
            polygons = []
            for obs in rvg_obstacles:
                for i in range(obs.size()):
                    p = polygon1.copy()
                    p.move_to([obs.getX()[i], obs.getY()[i], 0])
                    polygons.append(p)
            
            for i in range(rvg_boundary.size()):
                p = polygon1.copy()
                p.move_to([rvg_boundary.getX()[i], rvg_boundary.getY()[i], 0])
                polygons.append(p)


            polygon_group = VGroup(*polygons)
            grown_obs = layer.getGrownObs()
            grown_obs_group = VGroup(*[convert_rvgPolygon_to_manimPolygon(obs, opacity=1.0) for obs in grown_obs])

            shrunk_map = layer.getShrinkedBorder()
            shrunk_map = convert_rvgPolygon_to_manimPolygon(shrunk_map, 0.8, GREY_E, stroke_width=4)

            self.move_camera(
                frame_center=[0,0,0],
                zoom=1,
                added_anims=[
                FadeIn(map),
                FadeIn(obstacle),
                FadeTransform(polygon1, polygon_group),
                ],
                run_time=1
            )

            self.wait()
            self.add(shrunk_map)
            self.play(
                Transform(map, shrunk_map),
                FadeIn(grown_obs_group),
                FadeOut(obstacle),
                FadeOut(polygon_group),
                run_time=1
            )
            self.remove(map)

        with self.voiceover(text="Given a resolution n, \
                            the rotational space can be sliced into multiple layers with coresponding rotation ranges,\
                            and we can build Visibility Graphs for each layer") as tracker:
            self.move_camera(
                phi=75*DEGREES,
                theta=30*DEGREES,
                frame_center=[0,0,0],
                zoom=1,
                added_anims=[
                ],
                run_time=1
            )
            axes = ThreeDAxes(z_range=[-0.1, PI+0.1])
            self.add(axes)
            maps = VGroup(*[convert_rvgPolygon_to_manimPolygon(layers[i].getShrinkedBorder(), color=GREY_E, stroke_width=4, opacity=0.8).move_to([0, 0, PI*i/len(layers)]) for i in range(len(layers))])
            grown_obs_groups = []
            for i in range(len(layers)):
                layer = layers[i]
                grown_obs = layer.getGrownObs()
                for obs in grown_obs:
                    grown_obs_manim = convert_rvgPolygon_to_manimPolygon(obs, opacity=0.8)
                    center = grown_obs_manim.get_center()
                    center[2] = PI*i/len(layers)
                    grown_obs_manim.move_to(center)
                    grown_obs_groups.append(grown_obs_manim)

            grown_obs_groups = VGroup(*grown_obs_groups)

            self.play(
                Transform(shrunk_map, maps), 
                Create(axes),
                run_time=1
                )
            self.play(
                Transform(grown_obs_group[0], grown_obs_groups),
                run_time = 1
            )
            
        with self.voiceover(text="Now let's take the first two layers as an example and see how layers are connected into one graph.") as tracker:
            maps2 = VGroup(*[convert_rvgPolygon_to_manimPolygon(layers[i].getShrinkedBorder(), color=GREY_E, stroke_width=4, opacity=0).move_to([0, 0, PI*i/len(layers)]) for i in range(2)])
            grown_obs_groups_2 = []
            dots_layers = []
            for i in range(2):
                layer = layers[i]
                grown_obs = layer.getGrownObs()
                for obs in grown_obs:
                    grown_obs_manim = convert_rvgPolygon_to_manimPolygon(obs, 0.8)
                    center = grown_obs_manim.get_center()
                    center[2] = PI*i/len(layers)
                    grown_obs_manim.move_to(center)
                    grown_obs_groups_2.append(grown_obs_manim)
                    dots = getVertices(obs)
                    dots_layers.append([Dot3D(point=[dot[0], dot[1], PI*i/len(layers)+0.02], color=PINK, radius=0.04, fill_opacity=1.0) for dot in dots])
            
            dots_layers = VGroup(*[VGroup(*dots) for dots in dots_layers])

            grown_obs_groups_2 = VGroup(*grown_obs_groups_2)
            self.move_camera(
                frame_center=[0,0,0],
                zoom=2,
                added_anims=[
                    FadeOut(shrunk_map),
                    FadeOut(maps),
                    FadeOut(grown_obs_group),
                    FadeIn(maps2),
                    FadeIn(grown_obs_groups_2),
                ],
                run_time=1
            )

            self.play(
                FadeIn(dots_layers),
                run_time=1
            )
            self.remove(shrunk_map, grown_obs_group)   
        
        with self.voiceover(text="In each layer, the visible area of any vertices can be easily queried.") as tracker:
            dots = []
            for i in range(2):
                layer = layers[i]
                dots_layer = []
                for (x, y, _) in getVertices(layer.getGrownObs()[0]):
                    if x>0 and y>0:
                        dots_layer.append(Dot3D(point=[x, y, PI*i/len(layers) + 0.02], color=PINK, radius=0.06, fill_opacity=1.0))
                dots.append(dots_layer)
                

            self.play(
                FadeOut(dots_layers),
                run_time=1
            )
            
            for j in range(2):
                visible_areas = []
                dots0 = dots[j]
                for i in range(len(dots0)):
                    if i == 0:
                        x, y = dots0[i].get_center()[:2] 
                        visible_area = layers[j].getVisibleAreaPolygon(x, y)
                        visible_area = convert_rvgPolygon_to_manimPolygon(visible_area, opacity=0.8, color=GOLD, stroke_width=1, z=PI*j/len(layers))
                        visible_areas.append(visible_area)
                        self.play(
                            FadeIn(dots0[i]),
                            FadeIn(visible_area),
                            run_time=1
                        )
                    else:
                        x, y = dots0[i].get_center()[:2] 
                        visible_area = layers[j].getVisibleAreaPolygon(x, y)
                        visible_area = convert_rvgPolygon_to_manimPolygon(visible_area, opacity=0.8, color=GOLD, stroke_width=1, z=PI*j/len(layers))
                        visible_areas.append(visible_area)
                        self.play(
                            FadeOut(visible_areas[i-1]),
                            FadeOut(dots0[i-1]),
                            FadeIn(dots0[i]),
                            FadeIn(visible_area),
                            run_time=1
                        )
                self.play(
                    FadeOut(dots0[-1]),
                    FadeOut(visible_areas[-1]),
                    run_time=1
                )

        with self.voiceover(text="If vertex A from layer 1 falls into the visible area of vertex B from layer 2,") as tracker:
            dot0 = dots[0][1]
            dot1 = dots[1][1] 
            visible_area = layers[1].getVisibleAreaPolygon(*dot1.get_center()[:2])
            visible_area = convert_rvgPolygon_to_manimPolygon(visible_area, opacity=0.8, color=GOLD, stroke_width=1, z=PI/len(layers))
            a = Tex(r"B", font_size=40)
            b = Tex(r"A", font_size=40)
            self.add_fixed_in_frame_mobjects(a, b)
            a.move_to([0.5, 1.2, 0])
            b.move_to([1.24, -1.1, 0])
            self.play(
                FadeIn(dot0),
                FadeIn(dot1),
                Create(a),
                Create(b),
                run_time=1
            )

            segment0 = dot0.get_center()
            segment1 = dot0.get_center()
            dot0_1 = dot0.copy()
            segment1[2] = PI/len(layers)
            dot0_1.move_to(segment1)
            line = Line(start = segment0, end=segment1, color=PINK)
            self.play(
                Create(line),
                FadeIn(visible_area),
                FadeIn(dot0_1),
                run_time=1
            )
        with self.voiceover(text="and their connection is bi-tangent, then we can add an edge between A and B.") as tracker:
            edge = Line(start = dot0.get_center(), end=dot1.get_center(), color=PINK)
            self.play(
                FadeOut(line),
                FadeOut(dot0_1),
                FadeOut(visible_area),
                Create(edge), 
                run_time=1
            )
        
        with self.voiceover(text="Since A is in the free space of layer 2, \
                            we can add A to layer 2 as A prime so it can be checked with layer 3") as tracker:
            a_prime = Tex(r"A'", font_size=40)
            self.add_fixed_in_frame_mobjects(a_prime)
            a_prime.move_to([1.24, 1.2, 0])
            self.add(a_prime)
            self.play(
                Create(a_prime),
                FadeIn(dot0_1),
                run_time=1
            )
        
        with self.voiceover(text="Now we can check all the vertices in layer 1 with layer 2") as tracker:
            self.play(
                FadeOut(a_prime),
                FadeOut(dot0_1),
                FadeOut(a),
                FadeOut(b),
                FadeOut(edge),
                FadeOut(dot0),
                FadeOut(dot1),
                run_time=1
            )

            lines = []
            dots = []
            for edge in layerEdgesForward[0]:
                v0 = edge[0]
                v1 = edge[1]
                if v0.getTheta() > v1.getTheta():
                    v0, v1 = v1, v0

                dot0 = Dot3D(point=[v0.getX(), v0.getY(), 0], color=PINK, radius=0.06, fill_opacity=1.0)
                dot1 = Dot3D(point=[v1.getX(), v1.getY(), PI/len(layers)], color=PINK, radius=0.06, fill_opacity=1.0)
                dots.append(dot0)
                dots.append(dot1)

                line = Line(start = dot0.get_center(), end=dot1.get_center(), color=PINK)
                line.set_stroke(opacity=0.5)
                lines.append(line)
                
            lines = VGroup(*lines)
            dots = VGroup(*dots)
            self.play(
                FadeIn(dots),
                Create(lines),
                run_time=2
            )
        
        with self.voiceover(text="We can also do this for the rest of the layers until the last layer is reached. We call this process as 'Forward Vertex Propagation'.") as tracker:
            maps = VGroup(*[convert_rvgPolygon_to_manimPolygon(layers[i].getShrinkedBorder(), color=GREY_E, stroke_width=4, opacity=0.0).move_to([0, 0, PI*i/len(layers)]) for i in range(2, len(layers))])
            grown_obs_groups = []
            for i in range(2, len(layers)):
                layer = layers[i]
                grown_obs = layer.getGrownObs()
                for obs in grown_obs:
                    grown_obs_manim = convert_rvgPolygon_to_manimPolygon(obs, opacity=0.8)
                    center = grown_obs_manim.get_center()
                    center[2] = PI*i/len(layers)
                    grown_obs_manim.move_to(center)
                    grown_obs_groups.append(grown_obs_manim)

            grown_obs_groups = VGroup(*grown_obs_groups)
            self.play(
                Create(maps),
                Create(grown_obs_groups),
                run_time = 1
            )

            dots_layers = []
            lines_layers = []
            for i in range(1, len(layers)):
                lines_layer = []
                dots_layer = []
                for edge in layerEdgesForward[i]:
                    v0 = edge[0]
                    v1 = edge[1]
                    if v0.getTheta() > v1.getTheta():
                        v0, v1 = v1, v0

                    dot0 = Dot3D(point=[v0.getX(), v0.getY(), PI*i/len(layers)], color=PINK, radius=0.06, fill_opacity=1.0)
                    dot1 = Dot3D(point=[v1.getX(), v1.getY(), PI*((i+1)%len(layers))/len(layers)], color=PINK, radius=0.06, fill_opacity=1.0)
                    dots_layer.append(dot0)
                    dots_layer.append(dot1)

                    line = Line(start = dot0.get_center(), end=dot1.get_center(), color=PINK)
                    line.set_stroke(opacity=0.5)
                    lines_layer.append(line)
                
                lines_layer = VGroup(*lines_layer)
                dots_layer = VGroup(*dots_layer)
                dots_layers.append(dots_layer)
                lines_layers.append(lines_layer)
                if i < len(layers) - 1:
                    self.move_camera(
                        frame_center=[0,0,PI*i/len(layers)],
                        zoom=2,
                        added_anims=[
                            Create(dots_layer),
                            Create(lines_layer),
                        ],
                        run_time=1
                    )
                else:
                    self.move_camera(
                        frame_center=[0,0,0],
                        zoom=1,
                        added_anims=[
                            Create(dots_layer),
                            Create(lines_layer),
                        ],
                        run_time=1
                    )

        with self.voiceover(text="In the same way, we can also do 'Backward Vertex Propagation' to make sure all the clockwise rotations are also enabled.") as tracker:
            self.play(
                FadeOut(dots),
                FadeOut(lines),
            )
            for i in range(len(dots_layers)):
                self.play(
                    FadeOut(dots_layers[i]),
                    FadeOut(lines_layers[i]),
                    run_time=1
                )

            lines_layers = []
            dots_layers = []
            for i in range(len(layers)-1, 0, -1):
                lines_layer = []
                dots_layer = []
                for edge in layerEdgesBackward[i]:
                    v0 = edge[0]
                    v1 = edge[1]
                    if v0.getTheta() < v1.getTheta():
                        v0, v1 = v1, v0

                    dot0 = Dot3D(point=[v0.getX(), v0.getY(), PI*i/len(layers)], color=PINK, radius=0.06, fill_opacity=1.0)
                    dot1 = Dot3D(point=[v1.getX(), v1.getY(), PI*((i-1)%len(layers))/len(layers)], color=PINK, radius=0.06, fill_opacity=1.0)
                    dots_layer.append(dot0)
                    dots_layer.append(dot1)

                    line = Line(start = dot0.get_center(), end=dot1.get_center(), color=PINK)
                    line.set_stroke(opacity=0.5)
                    lines_layer.append(line)
                
                lines_layer = VGroup(*lines_layer)
                dots_layer = VGroup(*dots_layer)
                dots_layers.append(dots_layer)
                lines_layers.append(lines_layer)
                if i > 0:
                    self.move_camera(
                        frame_center=[0,0,PI*i/len(layers)],
                        zoom=2,
                        added_anims=[
                            Create(dots_layer),
                            Create(lines_layer),
                        ],
                        run_time=1
                    )
                else:
                    self.move_camera(
                        frame_center=[0,0,0],
                        zoom=1,
                        added_anims=[
                            Create(dots_layer),
                            Create(lines_layer),
                        ],
                        run_time=1
                    )
        
        with self.voiceover(text="Now we have the whole RVG constructed.") as tracker:
            dots_layers_forward_all = []
            lines_layers_forward_all = []
            for i in range(1, len(layers)):
                for edge in layerEdgesForward[i]:
                    v0 = edge[0]
                    v1 = edge[1]
                    if v0.getTheta() > v1.getTheta():
                        v0, v1 = v1, v0

                    dot0 = Dot3D(point=[v0.getX(), v0.getY(), PI*i/len(layers)], color=PINK, radius=0.06, fill_opacity=1.0)
                    dot1 = Dot3D(point=[v1.getX(), v1.getY(), PI*((i+1)%len(layers))/len(layers)], color=PINK, radius=0.06, fill_opacity=1.0)
                    dots_layers_forward_all.append(dot0)
                    dots_layers_forward_all.append(dot1)

                    line = Line(start = dot0.get_center(), end=dot1.get_center(), color=PINK)
                    line.set_stroke(opacity=0.5)
                    lines_layers_forward_all.append(line)
            
            dots_layers_forward_all = VGroup(*dots_layers_forward_all)
            lines_layers_forward_all = VGroup(*lines_layers_forward_all)
            self.add(dots_layers_forward_all, lines_layers_forward_all)
            self.play(
                Create(dots_layers_forward_all),
                Create(lines_layers_forward_all),
                run_time=1
            )
            
        with self.voiceover(text="Given the start") as tracker:
            dot_start = Dot3D(point=[rvg_start.getX(), rvg_start.getY(), rvg_start.getTheta()], color=LIGHT_BROWN, radius=0.1, fill_opacity=1.0)
            dot_end = Dot3D(point=[rvg_goal.getX(), rvg_goal.getY(), rvg_goal.getTheta()], color=LIGHT_BROWN, radius=0.1, fill_opacity=1.0)
            self.move_camera(
                phi=90*DEGREES,
                theta=-90*DEGREES,
                frame_center=[0,0,PI/2],
                zoom=1.5,
                run_time=1
            )
            self.add(dot_start, dot_end)
            self.play(
                Indicate(dot_start),
                run_time=1
            )
        
        with self.voiceover(text="and goal") as tracker:
            self.play(
                Indicate(dot_end),
                run_time=1
            )
        
        with self.voiceover(text="we can add them to the RVG by connecting them with the visible vertices in RVG") as tracker:
            adjacent_start = vg.getAdjacentVertices(rvg_start)
            adjacent_end = vg.getAdjacentVertices(rvg_goal)
            start_lines = []
            end_lines = []
            for v in adjacent_start:
                line = Line(start = [rvg_start.getX(), rvg_start.getY(), rvg_start.getTheta()], end=[v.getX(), v.getY(), v.getTheta()-PI/len(layers)/2], color=LIGHT_BROWN)
                line.set_stroke(opacity=0.5)
                start_lines.append(line)

            for v in adjacent_end:
                line = Line(start = [rvg_goal.getX(), rvg_goal.getY(), rvg_goal.getTheta()], end=[v.getX(), v.getY(), v.getTheta()-PI/len(layers)/2], color=LIGHT_BROWN)
                line.set_stroke(opacity=0.5)
                end_lines.append(line)
            
            start_lines = VGroup(*start_lines)
            end_lines = VGroup(*end_lines)
            self.play(
                Create(start_lines),
                Create(end_lines),
                run_time=1
            )
        
        with self.voiceover(text="Now we can find the shortest path in the RVG") as tracker:
            sol = vg.shortestPath(rvg_start, rvg_goal, 10, False)
            points = []
            for i in range(len(sol)):
                v = sol[i]
                # dot = Dot3D(point=[v.getX(), v.getY(), v.getTheta()], color=LIGHT_BROWN, radius=0.1, fill_opacity=1.0)
                if i == 0:
                    point = [rvg_start.getX(), rvg_start.getY(), rvg_start.getTheta()]
                elif i == len(sol) - 1:
                    point = [rvg_goal.getX(), rvg_goal.getY(), rvg_goal.getTheta()]
                else:
                    point = [v.getX(), v.getY(), v.getTheta()-PI/len(layers)/2]
                points.append(point)

            path = VMobject(stroke_color=GOLD_E, stroke_width=10) 
            path.set_points_as_corners(points) 
            self.play(
                Create(path),
                run_time=3
            )
            self.remove(start_lines, end_lines, lines_layers_forward_all, dots_layers_forward_all)
            self.remove(*lines_layers, *dots_layers)


        sol = vg.shortestPath(rvg_start, rvg_goal, 10, True)
        robot = Rectangle(height = 1, width=0.5)
        robot.set_fill(PINK, opacity=0.8)
        robot.set_stroke(WHITE, 1)
        goal.rotate(rvg_goal.getTheta())
        robot.rotate(rvg_start.getTheta())
        robot.move_to([rvg_start.getX(), rvg_start.getY(), 0])
        map = Square(5)
        map.set_fill(GREY_E, opacity=0.8)
        obstacle = Square(1.5)
        obstacle.set_fill(GREEN, opacity=0.8)
        obstacle.set_stroke(WHITE, 1)
        start = Rectangle(height = 1, width=0.5)
        start.set_fill(PINK, opacity=0.8)
        start.set_stroke(WHITE, 1)
        start.rotate(rvg_start.getTheta())
        start.move_to([rvg_start.getX(), rvg_start.getY(), 0])

        goal = Rectangle(height = 1, width=0.5)
        goal.set_fill(PINK, opacity=0.8)
        goal.set_stroke(WHITE, 1)
        goal.rotate(rvg_goal.getTheta())
        goal.move_to([rvg_goal.getX(), rvg_goal.getY(), 0])
        self.add(map, obstacle, start, goal)

        with self.voiceover(text="Now we have the final solution") as tracker:
            self.move_camera(
                phi=0,
                theta=-90*DEGREES,
                frame_center=[0,0,0],
                zoom=1,
                added_anims=[
                    Transform(maps2, map),
                    FadeOut(maps),
                    FadeOut(grown_obs_groups),
                    FadeOut(grown_obs_groups_2),
                    FadeIn(obstacle),
                    FadeIn(start),
                    FadeIn(goal),
                ],
                run_time=1
            )
            for i in range(len(sol)):
                sol_i = sol[i]
                if i == 0:
                    angle = 0
                else:
                    angle = sol_i.getTheta() -  sol[i-1].getTheta()
                self.play(robot.animate.move_to([sol_i.getX(), sol_i.getY(), 0]).rotate(angle), run_time=0.1)

