"""
Pick and Place Example for Denso Cobotta Robot

This script demonstrates a complete pick-and-place task using the Denso Cobotta robot
with an OnRobot RG6 gripper in Isaac Sim. The robot picks up a cube from an initial
position and places it at a target location.

Flow:
1. Initialize Isaac Sim simulation environment
2. Create a pick-and-place task with target position
3. Load Denso robot with gripper into the scene
4. Initialize pick-and-place controller
5. Run simulation loop:
   - Get observations (robot state, object position)
   - Compute actions using controller
   - Apply actions to robot
   - Repeat until task completion

Key Components:
- PickPlace: Task that sets up the robot, cube, and target position
- PickPlaceController: High-level controller that manages the pick-and-place sequence
- World: Isaac Sim simulation world manager
"""
from isaacsim import SimulationApp

# ============================================================================
# 1. Initialize Isaac Sim Application
# ============================================================================
# Create the simulation app with GUI (headless=False shows the viewer)
simulation_app = SimulationApp({"headless": False})

# Import Isaac Sim modules AFTER SimulationApp is created
from isaacsim.core.api import World
from omni.isaac.core.objects import GroundPlane
import numpy as np
from custom_pick_place import CustomPickPlace
from pick_place_controller import PickPlaceController
from isaacsim.core.utils.stage import add_reference_to_stage

# ============================================================================
# 2. Create Simulation World
# ============================================================================
# Create world with 1 meter as base unit
my_world = World(stage_units_in_meters=1.0)

# Create ground plane with physics
ground_plane = my_world.scene.add(
    GroundPlane(
        prim_path="/World/GroundPlane",
        size=100.0,  # Size in meters
        color=None,  # Optional color (R, G, B)
        physics_material=None  # Optional physics material
    )
)
asset_path = "D:/poc/po_wiwynn_test/prp_table01.usda"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/Table")

# ============================================================================
# 3. Define Pick-and-Place Task
# ============================================================================
# Set the target position where the cube should be placed
# Position format: [x, y, z] in meters relative to world origin
initial_position = np.array([-0.5, 0.4, 0.125])
target_position = np.array([-0.5, -0.5, 0.5])
# Set Z to half the cube height (0.0515m) so cube sits on the ground
target_position[2] = 0.0515 / 2.0  # Z = 0.02575 meters


# Create the pick-and-place task
# This will:
# - Load the Denso Cobotta robot with OnRobot RG6 gripper
# - Create a cube at default initial position
# - Set up the target position for placing the cube
# my_task = PickPlace(name="denso_pick_place",
#                     cube_initial_position=initial_position,
#                     target_position=target_position)
my_task = CustomPickPlace(name="denso_pick_place",
                          object_type="usd",
                          custom_usd_path="D:/poc/po_wiwynn_test/tst_cylinder01.usda",  # Local or Omniverse path
                          object_initial_position=initial_position,
                          target_position=target_position
                          )


# ============================================================================
# 4. Initialize Scene and Robot
# ============================================================================
# Add the task to the world (registers robot and objects)
my_world.add_task(my_task)

# Reset the world to initialize physics and prepare the scene
my_world.reset()

# Get the robot object from the scene
# Name "cobotta_robot" is defined in PickPlace.set_robot()
my_denso = my_world.scene.get_object("cobotta_robot")

# ============================================================================
# 5. Initialize Controllers
# ============================================================================
# Create the high-level pick-and-place controller
# This controller manages the state machine for:
# - Moving to pre-grasp position
# - Approaching object
# - Closing gripper
# - Lifting object
# - Moving to target
# - Placing object
# - Opening gripper
my_controller = PickPlaceController(
    name="controller",
    robot_articulation=my_denso,
    gripper=my_denso.gripper
)

# Get task parameters (includes object names, robot name, etc.)
task_params = my_world.get_task("denso_pick_place").get_params()

# Get the low-level articulation controller for applying joint commands
articulation_controller = my_denso.get_articulation_controller()

# Counter variable (currently unused)
i = 0


# ============================================================================
# 6. Main Simulation Loop
# ============================================================================
while simulation_app.is_running():
    """
    Main simulation loop that runs every frame.

    Loop structure:
    1. Step the simulation forward (physics, rendering)
    2. Check if simulation is playing (not paused)
    3. Get observations from the world
    4. Compute actions from controller
    5. Apply actions to robot
    """

    # Step the simulation forward by one frame and render the scene
    my_world.step(render=True)

    # Only execute control logic when simulation is playing (not paused)
    if my_world.is_playing():

        # Reset on first time step (handles simulation restarts)
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()

        # ====================================================================
        # 6.1 Get Observations
        # ====================================================================
        # Collect current state information from the world:
        # - Robot joint positions
        # - Cube position and orientation
        # - Target position
        observations = my_world.get_observations()

        # ====================================================================
        # 6.2 Compute Actions
        # ====================================================================
        # Pass observations to the controller to get desired actions
        # The controller uses these inputs to determine the next robot motion
        actions = my_controller.forward(
            # Current position of the cube to pick up
            picking_position=observations[task_params["cube_name"]["value"]]["position"],

            # Target position where to place the cube
            placing_position=observations[task_params["cube_name"]["value"]]["target_position"],

            # Current joint angles of the robot (6 arm joints + gripper joints)
            current_joint_positions=observations[task_params["robot_name"]["value"]]["joint_positions"],

            # Offset between end-effector and gripper center point [x, y, z]
            # This needs tuning based on gripper geometry
            # Positive Z offset moves the grasp point up
            end_effector_offset=np.array([0, 0, 0.25]),
        )

        # ====================================================================
        # 6.3 Check Task Completion
        # ====================================================================
        # Check if the pick-and-place sequence is complete
        if my_controller.is_done():
            print("done picking and placing")

        # ====================================================================
        # 6.4 Apply Actions to Robot
        # ====================================================================
        # Send computed actions (joint positions/velocities) to the robot
        articulation_controller.apply_action(actions)

# ============================================================================
# 7. Cleanup
# ============================================================================
# Close the simulation app when done
simulation_app.close()