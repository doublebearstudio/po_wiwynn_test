"""
Configuration file for Robot Pick and Place Controller
Adjust these parameters to customize the behavior without modifying the main scripts.
"""

# ===========================
# FILE PATHS
# ===========================

# Path to the USD stage file
USD_STAGE_PATH = "D:/poc/po_wiwynn_test/po_wiwynn_test_v0002.usda"


# ===========================
# ROBOT CONFIGURATION
# ===========================

# Robot prim paths in the USD stage
ROBOT1_PATH = "/World/kr210_l150_01"
ROBOT2_PATH = "/World/kr210_l150_02"

# End effector names
ROBOT1_EE_NAME = "tool0"
ROBOT2_EE_NAME = "tool0"


# ===========================
# OBJECT PATHS
# ===========================

# Objects to be picked
BOX_LID_PATH = "/World/geo_boxLid_01"
CUBE_PATH = "/World/geo_cube_01"

# Target locations
DANNY_TABLE_PATH = "/World/Danny"
BOX_BODY_PATH = "/World/grp_boxBody_01"


# ===========================
# MOTION PARAMETERS
# ===========================

# Number of steps for smooth motion (higher = slower/smoother)
MOTION_STEPS = {
    "approach": 150,      # Steps to approach position
    "grasp": 100,         # Steps to grasp position
    "lift": 100,          # Steps to lift object
    "place": 100,         # Steps to place object
    "retract": 100,       # Steps to retract after placing
    "home": 200,          # Steps to return home
}

# Approach heights (meters) - how high above object/target to approach
APPROACH_HEIGHTS = {
    "pick": 0.30,         # Height above object before picking
    "place": 0.30,        # Height above target before placing
}

# Grasp offset (meters) - offset from object center when grasping
GRASP_OFFSET_Z = 0.05     # Grasp slightly above object center


# ===========================
# PLACEMENT OFFSETS
# ===========================

# Offset for placing box lid on Danny table (meters)
DANNY_PLACEMENT_OFFSET = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.40,            # Place this high above Danny table surface
}

# Offset for placing cube in box body (meters)
BOX_BODY_PLACEMENT_OFFSET = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.20,            # Place this high inside box body
}


# ===========================
# GRIPPER CONFIGURATION
# ===========================

# Gripper wait time (seconds) - time to wait for gripper to open/close
GRIPPER_WAIT_TIME = 0.5

# Gripper joint limits (if using actual gripper control)
GRIPPER_OPEN_POSITION = 0.04      # Open position (meters)
GRIPPER_CLOSE_POSITION = 0.0      # Close position (meters)

# Gripper force (Newtons) - for force-controlled grippers
GRIPPER_FORCE = 50.0


# ===========================
# SIMULATION SETTINGS
# ===========================

# Simulation configuration
SIMULATION_CONFIG = {
    "headless": False,              # Set True for no GUI (faster)
    "width": 1920,                  # Window width
    "height": 1080,                 # Window height
}

# Physics timestep
PHYSICS_DT = 1.0 / 60.0            # 60 Hz physics update

# Render frequency (render every N steps)
RENDER_FREQUENCY = 1               # 1 = render every step, 2 = every other step


# ===========================
# SAFETY LIMITS
# ===========================

# Maximum joint velocities (rad/s)
MAX_JOINT_VELOCITY = 1.0

# Maximum end effector velocity (m/s)
MAX_EE_VELOCITY = 0.5

# Collision check enabled
ENABLE_COLLISION_CHECK = True


# ===========================
# DEBUGGING OPTIONS
# ===========================

# Print verbose debug information
DEBUG_MODE = True

# Visualize target positions (requires debug draw extension)
VISUALIZE_TARGETS = False

# Pause after each major step (requires user input to continue)
PAUSE_BETWEEN_STEPS = False

# Log joint positions at each step
LOG_JOINT_POSITIONS = False


# ===========================
# CAMERA SETTINGS
# ===========================

# Default camera view position
CAMERA_VIEW = {
    "eye": [3.0, 3.0, 2.5],        # Camera position
    "target": [0.0, 0.0, 1.0],     # Look at point
}


# ===========================
# TASK SEQUENCE CONFIGURATION
# ===========================

# Enable/disable individual tasks
ENABLE_TASKS = {
    "task1_robot2_boxlid": True,   # Robot 2 picks box lid to Danny
    "task2_robot1_cube": True,     # Robot 1 picks cube to box body
    "task3_return_home": True,     # Both robots return home
}

# Delay between tasks (seconds)
INTER_TASK_DELAY = 1.0


# ===========================
# HELPER FUNCTIONS
# ===========================

def get_motion_steps(motion_type):
    """Get number of steps for a motion type"""
    return MOTION_STEPS.get(motion_type, 100)


def get_approach_height(operation):
    """Get approach height for pick or place operation"""
    return APPROACH_HEIGHTS.get(operation, 0.3)


def get_placement_offset(target_name):
    """Get placement offset for a target"""
    if "danny" in target_name.lower():
        return DANNY_PLACEMENT_OFFSET
    elif "box" in target_name.lower():
        return BOX_BODY_PLACEMENT_OFFSET
    else:
        return {"x": 0.0, "y": 0.0, "z": 0.1}


def print_config():
    """Print current configuration"""
    print("\n" + "="*70)
    print(" "*20 + "CURRENT CONFIGURATION")
    print("="*70)
    print(f"\nUSD Stage: {USD_STAGE_PATH}")
    print(f"\nRobot 1: {ROBOT1_PATH}")
    print(f"Robot 2: {ROBOT2_PATH}")
    print(f"\nMotion Steps (approach): {MOTION_STEPS['approach']}")
    print(f"Approach Height (pick): {APPROACH_HEIGHTS['pick']} m")
    print(f"Approach Height (place): {APPROACH_HEIGHTS['place']} m")
    print(f"\nDanny Placement Offset Z: {DANNY_PLACEMENT_OFFSET['z']} m")
    print(f"Box Body Placement Offset Z: {BOX_BODY_PLACEMENT_OFFSET['z']} m")
    print(f"\nDebug Mode: {DEBUG_MODE}")
    print(f"Headless: {SIMULATION_CONFIG['headless']}")
    print("="*70 + "\n")


if __name__ == "__main__":
    # Print configuration when run directly
    print_config()
