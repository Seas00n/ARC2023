from pydrake.all import (getDrakePath, DiagramBuilder, SceneGraph, FindResourceOrThrow,
                         MultibodyPlant, Parser, RigidTransform, CoulombFriction, Box,
                         GeometryFrame, GeometryInstance, MakePhongIllustrationProperties, Sphere,
                         LogVectorOutput, StartMeshcat, MeshcatVisualizer, MeshcatVisualizerParams,
                         plot_system_graphviz,
                         Simulator, Diagram)

import numpy as np


class Quad_Station(Diagram):
    scene_graph: SceneGraph
    plant: MultibodyPlant

    def __init__(self):
        Diagram.__init__(self)
        self.set_name("Quad Station")
        self.builder = DiagramBuilder()
        self.scene_graph = self.builder.AddSystem(SceneGraph())
        self.scene_graph.set_name("scene_graph")
        self.plant = self.builder.AddSystem(MultibodyPlant(time_step=5e-3))
        self.plant.RegisterAsSourceForSceneGraph(self.scene_graph)
        self.plant.set_name("plant")
        self.parser = Parser(self.plant)

    def AddDog(self, model_urdf):
        self.quad = self.parser.AddModelFromFile(model_urdf, 'quad')

    def AddGround(self):
        surface_friction = CoulombFriction(
            static_friction=1.0,
            dynamic_friction=1.0
        )
        self.plant.RegisterCollisionGeometry(
            self.plant.world_body(),
            RigidTransform(),
            Box(50, 50, 0.001),
            "ground_collision",
            surface_friction
        )
        self.plant.RegisterVisualGeometry(
            self.plant.world_body(),
            RigidTransform(),
            Box(50, 50, 0.001),
            "ground_visual",
            np.array([0.5, 0.5, 0.5, 0.0])
        )

    def Finalize(self):
        self.plant.Finalize()

    def RegisterDogFrame(self):
        self.trunk_source = self.scene_graph.RegisterSource("trunk")
        self.trunk_frame = GeometryFrame("trunk")
        self.scene_graph.RegisterFrame(self.trunk_source, self.trunk_frame)  # 在躯干创建坐标系
        trunk_shape = Box(0.4, 0.2, 0.1)
        trunk_color = np.array([0.1, 0.1, 0.1, 0.4])
        X_trunk = RigidTransform()
        trunk_geometry = GeometryInstance(X_trunk, trunk_shape, "trunk")
        trunk_geometry.set_illustration_properties(MakePhongIllustrationProperties(trunk_color))
        self.scene_graph.RegisterGeometry(self.trunk_source, self.trunk_frame.id(), trunk_geometry)
        self.trunk_frame_ids = {"trunk": self.trunk_frame.id()}
        for foot in ["lf", "rf", "lh", "rh"]:
            foot_frame = GeometryFrame(foot)
            self.scene_graph.RegisterFrame(self.trunk_source, foot_frame)
            foot_shape = Sphere(0.02)
            X_foot = RigidTransform()
            foot_geometry = GeometryInstance(X_foot, foot_shape, foot)
            foot_geometry.set_illustration_properties(MakePhongIllustrationProperties(trunk_color))
            self.scene_graph.RegisterGeometry(self.trunk_source, foot_frame.id(), foot_geometry)
            self.trunk_frame_ids[foot] = foot_frame.id()

    def ConnectToMeshcatVisualizer(self):
        self.meshcat = StartMeshcat()
        self.visualizer = MeshcatVisualizer.AddToBuilder(
            self.builder,
            self.scene_graph,
            self.meshcat,
            MeshcatVisualizerParams()
        )
