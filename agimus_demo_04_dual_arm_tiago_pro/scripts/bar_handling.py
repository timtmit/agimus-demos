from math import sqrt
from rostools import process_xacro
from helper import Helper
from hpp.corbaserver import loadServerPlugin
from hpp.corbaserver.manipulation import (
    Client,
    ConstraintGraph,
    ConstraintGraphFactory,
    Constraints,
    ProblemSolver,
    Robot,
)
from hpp.gepetto.manipulation import ViewerFactory


#### Definition of object models ####
class Table:
    rootJointType = "anchor"
    urdfFilename = "package://agimus_demo_04_dual_arm_tiago_pro/urdf/table.urdf"
    srdfFilename = "package://agimus_demo_04_dual_arm_tiago_pro/srdf/table.srdf"


class Plate:
    rootJointType = "freeflyer"
    urdfFilename = "package://agimus_demo_04_dual_arm_tiago_pro/urdf/plate.urdf"
    srdfFilename = "package://agimus_demo_04_dual_arm_tiago_pro/srdf/plate.srdf"


class ReinforcmentBar:
    rootJointType = "freeflyer"
    urdfFilename = (
        "package://agimus_demo_04_dual_arm_tiago_pro/urdf/reinforcment_bar.urdf"
    )
    srdfFilename = (
        "package://agimus_demo_04_dual_arm_tiago_pro/srdf/reinforcment_bar.srdf"
    )


loadServerPlugin("corbaserver", "manipulation-corba.so")
Client().problem.resetProblem()

#### Definition of the robot model ####

urdf_xacro = "package://tiago_pro_description/robots/tiago_pro.urdf.xacro"
srdf_xacro = "package://tiago_pro_moveit_config/config/srdf/tiago_pro.srdf.xacro"
Robot.urdfString = process_xacro(
    urdf_xacro,
    "end_effector_left:=pal-pro-gripper",
    "end_effector_right:=pal-pro-gripper",
).replace("file://", "")
Robot.srdfString = ""
srdfString = process_xacro(
    srdf_xacro,
    "end_effector_left:=pal-pro-gripper",
    "end_effector_right:=pal-pro-gripper",
)
# Additional collision pairs to remove
# remove </robot> from file
i = srdfString.find("</robot>")
assert i != -1
srdfString = srdfString[:i]
for l1, l2 in [
    ("base_link", "wheel_front_left_link"),
    ("base_link", "wheel_front_right_link"),
    ("base_link", "wheel_rear_left_link"),
    ("base_link", "wheel_rear_right_link"),
    ("gripper_left_screw_left_link", "gripper_left_fingertip_left_link"),
    ("gripper_right_screw_left_link", "gripper_right_fingertip_left_link"),
]:
    srdfString += f'  <disable_collisions link1="{l1}" link2="{l2}" reason="Never"/>\n'
srdfString += "</robot>"
# Create robot instance from URDF and SRDF strings edited above
robot = Robot("tiago_pro-manip", "tiago_pro", rootJointType="planar")
robot.client.manipulation.robot.insertRobotSRDFModelFromString("tiago_pro", srdfString)

#### Problem Solver and Viewer ####

ps = ProblemSolver(robot)
vf = ViewerFactory(ps)
vf.loadObjectModel(Table, "table")
vf.loadObjectModel(Plate, "plate")
vf.loadObjectModel(ReinforcmentBar, "reinforcment_bar")

# ps.selectPathValidation("NoValidation", 1)

# Set joint bounds
robot.setJointBounds(
    "plate/root_joint", [-0.5, 1.5, -1, 1, 0, 1.5, -1, 1, -1, 1, -1, 1, -1, 1]
)
robot.setJointBounds(
    "reinforcment_bar/root_joint",
    [-0.5, 1.5, -1, 1, 0, 1.5, -1, 1, -1, 1, -1, 1, -1, 1],
)
robot.setJointBounds("tiago_pro/root_joint", [-1.5, 3.5, -2, 2, -1, 1, -1, 1])

#### Define Grippers and Handles ####
# Grippers on tiago
c = sqrt(2) / 2
robot.client.manipulation.robot.addGripper(
    "tiago_pro/arm_left_7_link", "tiago_pro/left", [0, 0, 0.19, 0, -c, 0, c], 0.02
)
robot.client.manipulation.robot.addGripper(
    "tiago_pro/arm_right_7_link", "tiago_pro/right", [0, 0, 0.19, 0, -c, 0, c], 0.02
)

# Handles on reinforcment bar
robot.client.manipulation.robot.addHandle(
    "reinforcment_bar/base_link",
    "reinforcment_bar/left",
    [0, 0.01, -0.25, 0, 0, -c, c],
    0.05,
    6 * [True],
)
robot.client.manipulation.robot.addHandle(
    "reinforcment_bar/base_link",
    "reinforcment_bar/right",
    [0, 0.01, 0.25, 0, 0, -c, c],
    0.05,
    6 * [True],
)

#### Create lists of locked joints ####

# 1. Lock Grippers (we don't care about gripper motion during planning)
lockedGrippers = {
    "tiago_pro/gripper_left_finger_joint": 0.05,
    "tiago_pro/gripper_left_inner_finger_left_joint": -0.05,
    "tiago_pro/gripper_left_fingertip_left_joint": 0.05,
    "tiago_pro/gripper_left_finger_right_joint": 0,
    "tiago_pro/gripper_left_inner_finger_right_joint": -0.05,
    "tiago_pro/gripper_left_fingertip_right_joint": 0.05,
    "tiago_pro/gripper_left_outer_finger_left_joint": -0.05,
    "tiago_pro/gripper_left_outer_finger_right_joint": -0.05,
    "tiago_pro/gripper_right_finger_joint": 0.05,
    "tiago_pro/gripper_right_inner_finger_left_joint": -0.05,
    "tiago_pro/gripper_right_fingertip_left_joint": 0.05,
    "tiago_pro/gripper_right_finger_right_joint": 0,
    "tiago_pro/gripper_right_inner_finger_right_joint": -0.05,
    "tiago_pro/gripper_right_fingertip_right_joint": 0.05,
    "tiago_pro/gripper_right_outer_finger_left_joint": -0.05,
    "tiago_pro/gripper_right_outer_finger_right_joint": -0.05,
}
locked_grippers = list()
for j, v in lockedGrippers.items():
    constraint = f"locked_{j}"
    ps.createLockedJoint(constraint, j, [v])
    locked_grippers.append(constraint)

# 2. Lock Head
lockedHead = {"tiago_pro/head_1_joint": 0, "tiago_pro/head_2_joint": 0}
locked_head = list()
for j, v in lockedHead.items():
    constraint = f"locked_{j}"
    ps.createLockedJoint(constraint, j, [v])
    locked_head.append(constraint)

# 3. Lock Wheels
locked_wheels = list()
for j in [
    "tiago_pro/wheel_front_left_joint",
    "tiago_pro/wheel_front_right_joint",
    "tiago_pro/wheel_rear_left_joint",
    "tiago_pro/wheel_rear_right_joint",
]:
    constraint = f"locked_{j}"
    ps.createLockedJoint(constraint, j, [1, 0])
    locked_wheels.append(constraint)
    ps.setConstantRightHandSide(constraint, True)

# 4. Lock Plate (FIXED on the table)
ps.createLockedJoint(
    "locked_plate/root_joint", "plate/root_joint", [0.6, 0, 0.66, 0, 0, 0, 1]
)
ps.setConstantRightHandSide("locked_plate/root_joint", True)
locked_plate = ["locked_plate/root_joint"]

# 5. Lock Base Mobility (Dynamic)
# Used to freeze the base during manipulation
ps.createLockedJoint("locked_base_mobility", "tiago_pro/root_joint", [0, 0, 1, 0])
ps.setConstantRightHandSide("locked_base_mobility", False)
locked_base_mobility = ["locked_base_mobility"]
# 6. Lock Arms (Dynamic for transit)
# IMPORTANT: EXCLUDE torso_lift_joint to allow height compensation between table and plate
locked_arms = list()
seen_joints = set()
for j in filter(
    lambda s: s.startswith("tiago_pro/")
    and not s.startswith("tiago_pro/head")
    and not s.startswith("tiago_pro/wheel")
    and "gripper" not in s
    and s != "tiago_pro/torso_lift_joint"  # <--- Excluded
    and s != "tiago_pro/root_joint",
    robot.jointNames,
):
    if j in seen_joints:
        continue
    seen_joints.add(j)
    constraint = f"locked_{j}"
    ps.createLockedJoint(constraint, j, [0])
    locked_arms.append(constraint)
    ps.setConstantRightHandSide(constraint, False)

# 7. Lock Torso (Dynamic)
ps.createLockedJoint("locked_torso", "tiago_pro/torso_lift_joint", [0])
ps.setConstantRightHandSide("locked_torso", False)
locked_torso = ["locked_torso"]

#### Constraint Graph ####

# Generate constraint graph
cg = ConstraintGraph(robot, "graph")  # Empty graph
factory = ConstraintGraphFactory(
    cg
)  # Factory to create the graph structure for manipulation
factory.setGrippers(
    ["tiago_pro/left"]
)  # Grippers used for manipulation (don't add the right here because if you do that the factory generates a graph for single arm manipulation and bimanual manipulation)
factory.environmentContacts(
    ["table/reinforcment_bar_support", "plate/top"]
)  # Surfaces of the environment used for contacts
factory.setObjects(
    ["reinforcment_bar"],
    [["reinforcment_bar/left"]],
    [
        ["reinforcment_bar/bottom"]
    ],  # arg1: names of manipulated objects, arg2: list of handles of each object for the grippers, arg3: list of contact surfaces with the environment
)  # Manipulated objects and their handles definition
factory.generate()  # Generate the graph structure

#### Additional states, transitions and constraints ####

# Add a state and transition to project on 'free' with static tiago_pro and plate
cg.createNode("unconstrained")
cg.createEdge(
    "unconstrained", "free", "project-on-free", 1, "unconstrained"
)  # (origine, destination, nom de la transition, poids, esace des solution dont elle doit aussi faire partie
# Lock wheels, head and grippers everywhere
cg.addConstraints(
    graph=True,
    constraints=Constraints(
        numConstraints=locked_grippers + locked_head + locked_wheels + locked_plate
    ),
)

# ADD gripper right to the graph for bimanual manipulation
# Add other pregrasp-grasp constraint in pregrasp|intersec|preplace states
g = "tiago_pro/right"
h = "reinforcment_bar/right"
cg.createGrasp(f"{g} grasps {h}", g, h)
cg.createPreGrasp(f"{g} pregrasps {h}", g, h)
cg.addConstraints(
    node="tiago_pro/left > reinforcment_bar/left | f_pregrasp",
    constraints=Constraints(
        numConstraints=["tiago_pro/right pregrasps reinforcment_bar/right"]
    ),
)
cg.addConstraints(
    node="tiago_pro/left > reinforcment_bar/left | f_intersec",
    constraints=Constraints(
        numConstraints=["tiago_pro/right grasps reinforcment_bar/right"]
    ),
)
cg.addConstraints(
    node="tiago_pro/left > reinforcment_bar/left | f_preplace",
    constraints=Constraints(
        numConstraints=["tiago_pro/right grasps reinforcment_bar/right"]
    ),
)
cg.addConstraints(
    node="tiago_pro/left grasps reinforcment_bar/left",
    constraints=Constraints(
        numConstraints=["tiago_pro/right grasps reinforcment_bar/right"]
    ),
)


node_grasp = "tiago_pro/left grasps reinforcment_bar/left"
# Edges transitions for transit states
# create an edge to allow the robot to move is base in grasp but not its arms
cg.createEdge("free", "free", "transit_free", 1, "free")
cg.addConstraints(
    edge="transit_free",
    constraints=Constraints(numConstraints=locked_arms + locked_torso),
)
# On s'assure que l'objet ne glisse pas sur la table pendant que le robot roule
cg.addConstraints(
    edge="transit_free",
    constraints=Constraints(numConstraints=["place_reinforcment_bar/complement"]),
)

# Transit dans l'état GRASP (avec object)
cg.createEdge(node_grasp, node_grasp, "transit_grasp", 1, node_grasp)
cg.addConstraints(
    edge="transit_grasp",
    constraints=Constraints(numConstraints=locked_arms + locked_torso),
)
# disable the base mobility during manipulation except during transit
# Liste des arêtes explicitement dédiées au movement de la base
explicit_transit_edges = ["transit_free", "transit_grasp", "project-on-free"]

# Définition des noeuds "Mobiles" (où le robot voyage)
mobile_nodes = ["free", "unconstrained"]


# for edge in cg.edges.keys():
#     # Ignore explicit transit edges
#     if edge in explicit_transit_edges:
#         continue

#     node_from, node_to = cg.getNodesConnectedByEdge(edge)

#     is_from_mobile = node_from in mobile_nodes
#     is_to_mobile = node_to in mobile_nodes
#     # Ignore Approach/Withdraw edges (Mobile <-> Static)
#     if is_from_mobile != is_to_mobile:
#         # print(f"Unlocked Approach/Withdraw: {edge}")
#         continue
#     # Lock base mobility in other cases
#     # Case A : "Loop | f" in Free
#     # Case B : Manipulation (Grasp -> Grasp, Pregrasp -> Grasp...)
#     cg.addConstraints(
#         edge=edge,
#         constraints=Constraints(numConstraints=locked_base_mobility)
#     )
#     print(f"Locked Base Mobility on {edge}")
cg.addConstraints(
    edge="Loop | f", constraints=Constraints(numConstraints=locked_base_mobility)
)
cg.addConstraints(
    edge="Loop | 0-0", constraints=Constraints(numConstraints=locked_base_mobility)
)
print(f"edge number: {len(cg.edges)}")
# Add weight to transitions
cg.setWeight("Loop | f", 1)
cg.setWeight("Loop | 0-0", 1)
cg.initialize()

#### Define Configuration ####

# Set initial configuration
q0 = robot.getCurrentConfig()
r = robot.rankInConfiguration["tiago_pro/root_joint"]
q0[r : r + 4] = [3, 0, -1, 0]
r = robot.rankInConfiguration["plate/root_joint"]
q0[r : r + 3] = [0.6, 0, 0.66]
r = robot.rankInConfiguration["reinforcment_bar/root_joint"]
q0[r : r + 7] = [
    1.20,
    -0.0009939583742700046,
    0.6680938848666721,
    0.12097379466237763,
    0.6966816640367284,
    0.6966816640367284,
    -0.12097379466237763,
]
res, q_init, err = cg.applyNodeConstraints(
    "free", q0
)  # Apply 'free' node constraints to q0 to get a valid initial configuration
q_goal = q_init[:]  # Copy initial configuration to goal configuration
q_goal[r : r + 7] = [
    0.2,
    0,
    0.7,
    0,
    c,
    c,
    0,
]  # Modify goal configuration for reinforcment_bar position
res, q_goal, err = cg.generateTargetConfig(
    "project-on-free", q_goal, q_goal
)  # Project goal configuration on 'free' node constraints
assert res
# Set goal configuration
# Load path optimizers
ps.loadPlugin("spline-gradient-based.so")
# ps.addPathOptimizer("SplineGradientBased_bezier3")
ps.selectPathProjector("Progressive", 0.1)
ps.addPathOptimizer("RandomShortcut")
ps.setInitialConfig(q_init)
ps.addGoalConfig(q_goal)

#### Generate intermediate configurations ####

helper = Helper(ps, cg)

# Get the key configurations
q1, q2 = helper.generateIntermediateConfigs(q_init, q_goal)
ps.addConfigToRoadmap(q1)
ps.addConfigToRoadmap(q2)

#### Solve ####
v = vf.createViewer()
v(q_init)
ps.solve()
helper.optimizePath(ps.numberPaths() - 1)
