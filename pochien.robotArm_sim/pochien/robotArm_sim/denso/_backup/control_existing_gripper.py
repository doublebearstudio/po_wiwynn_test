from isaacsim.core.api import World
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.core.utils.types import ArticulationAction
import numpy as np

# Get the existing World instance instead of creating a new one
my_world = World.instance()

# If World doesn't exist, create one
if my_world is None:
    my_world = World(stage_units_in_meters=1.0)

# Always reset to ensure physics is properly initialized
print("Resetting world to initialize physics...")
my_world.reset()
print("World initialized and ready!")

# Define the gripper using the existing robot in the stage
gripper = ParallelGripper(
    end_effector_prim_path="/World/cobotta_pro_900/onrobot_rg6_base_link",
    joint_prim_names=["finger_joint", "right_outer_knuckle_joint"],
    joint_opened_positions=np.array([0, 0]),
    joint_closed_positions=np.array([0.628, -0.628]),
    action_deltas=np.array([-0.628, 0.628]),
)

# Get the existing robot from the scene (may be None if not added programmatically)
my_denso = my_world.scene.get_object("cobotta_robot")

# If the robot was added with the gripper, you can access it directly
# Otherwise, you can control the gripper we just defined

# Example: Control the gripper
def control_gripper_demo():
    i = 0
    max_iterations = 1000

    # Ensure simulation is playing
    if not my_world.is_playing():
        my_world.play()
        print("Started simulation playback")

    while i < max_iterations:
        # Step the world first
        my_world.step(render=True)

        # Control the gripper
        gripper_positions = gripper.get_joint_positions()

        if i < 500:
            # Close the gripper slowly
            gripper.apply_action(
                ArticulationAction(joint_positions=[gripper_positions[0] + 0.1, gripper_positions[1] - 0.1]))
        else:
            # Open the gripper slowly
            gripper.apply_action(
                ArticulationAction(joint_positions=[gripper_positions[0] - 0.1, gripper_positions[1] + 0.1]))

        i += 1

    print("Gripper control demo completed!")

# Run the demo
control_gripper_demo()
