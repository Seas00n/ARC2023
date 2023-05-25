import matplotlib.pyplot as plt

from station import *
from simple import *
from basic_controller import *
from towr import *
robot_description_path = "./mini_cheetah/mini_cheetah_mesh.urdf"
drake_path = getDrakePath()
robot_description_file = "drake/" + os.path.relpath(
    robot_description_path, start=drake_path
)
robot_urdf = FindResourceOrThrow(robot_description_file)
station = Quad_Station()
station.AddGround()
station.AddDog(robot_urdf)
station.Finalize()
station.RegisterDogFrame()
station.ConnectToMeshcatVisualizer()
planner = station.builder.AddSystem(
    BasicTrunkPlanner(station.trunk_frame_ids))  # type:BasicTrunkPlanner
controller = station.builder.AddSystem(
    BasicController(station.plant, 5e-3, use_lcm=True))  # type:BasicController

station.builder.Connect(
    station.scene_graph.get_query_output_port(),
    station.plant.get_geometry_query_input_port())

station.builder.Connect(
    station.plant.get_geometry_poses_output_port(),
    station.scene_graph.get_source_pose_port(station.plant.get_source_id())
)
station.builder.Connect(
    planner.GetOutputPort("trunk_geometry"),
    station.scene_graph.get_source_pose_port(station.trunk_source)
)
station.builder.Connect(
    controller.GetOutputPort("quad_torques"),
    station.plant.get_actuation_input_port(station.quad)
)
station.builder.Connect(station.plant.get_state_output_port(),
                        controller.GetInputPort("quad_state"))
logger = LogVectorOutput(controller.GetOutputPort("output_metrics"), builder=station.builder)

diagram = station.builder.Build()  # type:Diagram
diagram.set_name("diagram")
diagram_context = diagram.CreateDefaultContext()
# plt.figure()
# plot_system_graphviz(diagram, max_depth=2)
# plt.show()


simulator = Simulator(diagram, diagram_context)
simulator.set_target_realtime_rate(1.0)
plant_context = diagram.GetMutableSubsystemContext(station.plant, diagram_context)
q0 = np.asarray([1.0, 0.0, 0.0, 0.0,  # base orientation
                 0.0, 0.0, 0.3,  # base position
                 0.0, -0.8, 1.6,
                 0.0, -0.8, 1.6,
                 0.0, -0.8, 1.6,
                 0.0, -0.8, 1.6])
qd0 = np.zeros(station.plant.num_velocities())
station.plant.SetPositions(plant_context, q0)
station.plant.SetVelocities(plant_context, qd0)
simulator.AdvanceTo(50)
