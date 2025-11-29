from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World
import numpy as np
from custom_pick_place import CustomPickPlace
from pick_place_controller import PickPlaceController

my_world = World(stage_units_in_meters=1.0)

# Set target position (where to place the object)
target_position = np.array([-0.3, 0.6, 0])
target_position[2] = 0.05  # 5 cm above ground

# Set initial object position (where object starts)
initial_position = np.array([0.3, 0.3, 0.05])

# Create custom pick and place task
# Choose object_type: "cylinder", "sphere", "cube", or "usd"
my_task = CustomPickPlace(
    name="custom_pick_place",
    object_type="cylinder",  # Change this to "sphere", "cube", or "usd" to try different objects
    object_initial_position=initial_position,
    target_position=target_position
)

my_world.add_task(my_task)
my_world.reset()

# Get robot and controller
my_denso = my_world.scene.get_object("cobotta_robot")
my_controller = PickPlaceController(
    name="controller",
    robot_articulation=my_denso,
    gripper=my_denso.gripper
)

task_params = my_world.get_task("custom_pick_place").get_params()
articulation_controller = my_denso.get_articulation_controller()

print(f"\n{'='*60}")
print(f"Custom Pick and Place Task Started")
print(f"{'='*60}")
print(f"Object type: {task_params['object_type']['value']}")
print(f"Object name: {task_params['cube_name']['value']}")
print(f"Robot name: {task_params['robot_name']['value']}")
print(f"Initial position: {initial_position}")
print(f"Target position: {target_position}")
print(f"{'='*60}\n")

i = 0
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()

        observations = my_world.get_observations()

        # Forward the observation values to the controller to get the actions
        actions = my_controller.forward(
            picking_position=observations[task_params["cube_name"]["value"]]["position"],
            placing_position=observations[task_params["cube_name"]["value"]]["target_position"],
            current_joint_positions=observations[task_params["robot_name"]["value"]]["joint_positions"],
            # This offset needs tuning based on your object size
            end_effector_offset=np.array([0, 0, 0.25]),
        )

        if my_controller.is_done():
            print("Done picking and placing!")

        articulation_controller.apply_action(actions)

simulation_app.close()
