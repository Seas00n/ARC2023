import os
import numpy as np
import pydrake
from pydrake.all import (AbstractValue, AddMultibodyPlantSceneGraph,
                         DiagramBuilder, JointSliders, LeafSystem,
                         MeshcatVisualizer, Parser, RigidTransform, JacobianWrtVariable,
                         RollPitchYaw, StartMeshcat, Role, MeshcatVisualizerParams)
from manipulation.scenarios import AddMultibodyTriad

meshcat = StartMeshcat()
test_mode = True if "TEST_SRCDIR" in os.environ else False


class PrintPose(LeafSystem):
    def __init__(self, plant, body_index):
        LeafSystem.__init__(self)
        self._body_index = body_index
        self._plant = plant
        self.DeclareAbstractInputPort("body_poses",
                                      AbstractValue.Make([RigidTransform()]))

        self.DeclareForcedPublishEvent(self.Publish)
        self.test_pose_array = []
        self.pose_counter = 0

    def Publish(self, context):
        pose = self.get_input_port().Eval(context)[self._body_index]
        if self.pose_counter < 30:
            print(pose)
            print("end position (m): " + np.array2string(
                pose.translation(), formatter={
                    'float': lambda x: "{:3.2f}".format(x)}))
            print("end roll-pitch-yaw (rad):" + np.array2string(
                RollPitchYaw(pose.rotation()).vector(),
                formatter={'float': lambda x: "{:3.2f}".format(x)}))
            self.test_pose_array.append(pose.GetAsMatrix4())
            print(pose.GetAsMatrix4())
            self.pose_counter += 1
        elif self.pose_counter == 30:
            np.save("pose.npy", self.test_pose_array)
            self.pose_counter += 1
        else:
            print("Enough pose")


class PrintJacobian(LeafSystem):
    def __init__(self, plant, frame):
        LeafSystem.__init__(self)
        self._plant = plant
        self._plant_context = plant.CreateDefaultContext()
        self._frame = frame
        self.DeclareVectorInputPort("state", plant.num_multibody_states())
        self.DeclareForcedPublishEvent(self.Publish)
        self.test_jacobian_array = []
        self.jacobian_counter = 0

    def Publish(self, context):
        state = self.get_input_port().Eval(context)
        self._plant_context = plant.CreateDefaultContext()
        self._plant.SetPositionsAndVelocities(self._plant_context, state)
        W = self._plant.world_frame()
        J_G = self._plant.CalcJacobianSpatialVelocity(
            self._plant_context, JacobianWrtVariable.kQDot, self._frame,
            [0, 0, 0], W, W)  ## This is the important line
        if self.jacobian_counter < 30:
            print("J_G:")
            print(np.array2string(J_G, formatter={
                'float': lambda x: "{:5.1f}".format(x)}))
            print(
                f"smallest singular value(J_G): {np.min(np.linalg.svd(J_G, compute_uv=False))}")
            self.test_jacobian_array.append(J_G)
            self.jacobian_counter += 1
        elif self.jacobian_counter == 30:
            np.save("jacobian.npy", self.test_jacobian_array)
        else:
            print("Enough Jacobian")


builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(
    builder, time_step=0.0
)
assert isinstance(plant, pydrake.multibody.plant.MultibodyPlant)
parser = Parser(plant)
parser.AddModels("./model.urdf")
base_frame = plant.GetFrameByName("base_link")
plant.WeldFrames(plant.world_frame(), base_frame)
AddMultibodyTriad(plant.GetFrameByName("end_effector"), scene_graph, 0.20, 0.008)
plant.Finalize()
plant_context = plant.CreateDefaultContext()
visualizer = MeshcatVisualizer.AddToBuilder(
    builder, scene_graph, meshcat)
robot_arm = plant.GetModelInstanceByName("robot_arm")
end_effector = plant.GetFrameByName("end_effector", robot_arm)
print_pose = builder.AddSystem(PrintPose(plant, end_effector.index()))
print_jacobian = builder.AddSystem(PrintJacobian(plant, end_effector))
builder.Connect(plant.get_body_poses_output_port(),
                print_pose.get_input_port())
builder.Connect(plant.get_state_output_port(),
                print_jacobian.get_input_port())
default_interactive_timeout = 1
sliders = builder.AddSystem(JointSliders(meshcat, plant))
diagram = builder.Build()
context = diagram.CreateDefaultContext()
diagram.ForcedPublish(context)
q1_array = np.linspace(0, np.pi * 2 / 3, 35)
q2_array = np.linspace(0, np.pi * 2 / 3, 35)
q3_array = np.linspace(0, np.pi * 2 / 3, 35)
q_list = np.vstack([q1_array, q2_array, q3_array])
np.save("q.npy", q_list)
for i in range(35):
    contex = diagram.CreateDefaultContext()
    plant.SetPositions(plant.GetMyContextFromRoot(contex),
                       plant.GetModelInstanceByName("robot_arm"),
                       [q1_array[i], q2_array[i], q3_array[i]])
    diagram.ForcedPublish(contex)

import modern_robotics as mr

pose_array = np.load("pose.npy")
jacobian_array = np.load("jacobian.npy")
q_array = np.load("q.npy")
q_array = q_array[:, 0:30]


