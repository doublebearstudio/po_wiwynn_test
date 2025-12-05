"""
Task Scheduler Examples

This file demonstrates how to use the TaskScheduler with various event controls.

Examples:
- Example 1: Two tasks with task1 disabled (won't execute even though created in scene)
- Example 2: Three tasks in sequence with pauses (task1 → 3s → task2 → 6s → task3)

Usage:
1. Open Isaac Sim
2. Open Script Editor (Window > Script Editor)
3. Copy/paste an example
4. Run it
5. Press PLAY in timeline

Author: Generated for Wiwynn Test
Date: 2025-12-03
"""

import sys
import numpy as np

# Add module path (adjust if needed)
module_path = r"D:/poc/po_wiwynn_test/pochien.robotArm_sim/pochien/robotArm_sim/denso"
if module_path not in sys.path:
    sys.path.append(module_path)

# Import modules
from simple_pick_place_setup import PickPlaceSceneSetup
from pick_place_controller import PickPlaceController
from task_scheduler import TaskScheduler

# Import Isaac Sim modules
from isaacsim.core.utils.stage import add_reference_to_stage
from omni.timeline import get_timeline_interface
import omni.timeline
import omni.physx as _physx


# ============================================================================
# EXAMPLE 1: Two Tasks with Task1 Disabled
# ============================================================================
def example_1_disabled_task():
    """
    Example 1: Two tasks with task1 disabled

    Demonstrates:
    - Creating two tasks in the scene
    - Disabling task1 so it won't execute during simulation
    - Only task2 will perform pick-and-place operation

    Expected behavior:
    - Both robots and objects are visible in the scene
    - When simulation plays, only task2's robot moves
    - Task1's robot remains idle
    """
    print("="*70)
    print("EXAMPLE 1: TWO TASKS WITH TASK1 DISABLED")
    print("="*70)

    # ========================================================================
    # 1. Define Task Configurations
    # ========================================================================
    task1_config = {
        "robot_position": np.array([1.0, 1.0, 0.0]),
        "initial_position": np.array([0.5, 1.4, 0.04]),
        "target_position": np.array([0.4, 0.5, 0.04]),
        "custom_usd_path": "D:/poc/po_wiwynn_test/tst_cylinder01.usda",
        "robot_prim_path": "/World/cobotta_task1",
        "object_prim_path": "/World/pickup_object_task1",
        "target_marker_path": "/World/target_marker_task1"
    }

    task2_config = {
        "robot_position": np.array([-1.0, 0.0, 0.0]),
        "initial_position": np.array([-1.6, 0.6, 0.04]),
        "target_position": np.array([-1.5, -0.2, 0.04]),
        "custom_usd_path": "D:/poc/po_wiwynn_test/tst_cylinder01.usda",
        "robot_prim_path": "/World/cobotta_task2",
        "object_prim_path": "/World/pickup_object_task2",
        "target_marker_path": "/World/target_marker_task2"
    }

    # ========================================================================
    # 2. Setup Scene - Create Both Tasks
    # ========================================================================
    print("\n→ Setting up scene with 2 tasks...")

    # Add table
    table_path = "D:/poc/po_wiwynn_test/prp_table02.usda"
    add_reference_to_stage(usd_path=table_path, prim_path="/World/Table")

    # Create Task 1
    print("  → Creating Task1 (WILL BE DISABLED)...")
    setup1 = PickPlaceSceneSetup(
        custom_usd_path=task1_config["custom_usd_path"],
        object_initial_position=task1_config["initial_position"],
        target_position=task1_config["target_position"],
        robot_position=task1_config["robot_position"],
        robot_prim_path=task1_config["robot_prim_path"],
        object_prim_path=task1_config["object_prim_path"]
    )
    setup1.target_marker_path = task1_config["target_marker_path"]
    robot1 = setup1.create_robot()
    setup1.create_object(mass=0.01)
    setup1.create_target_marker()

    controller1 = PickPlaceController(
        name="controller_task1",
        robot_articulation=robot1,
        gripper=robot1.gripper
    )
    articulation_controller1 = robot1.get_articulation_controller()

    # Create Task 2
    print("  → Creating Task2 (ENABLED)...")
    setup2 = PickPlaceSceneSetup(
        custom_usd_path=task2_config["custom_usd_path"],
        object_initial_position=task2_config["initial_position"],
        target_position=task2_config["target_position"],
        robot_position=task2_config["robot_position"],
        robot_prim_path=task2_config["robot_prim_path"],
        object_prim_path=task2_config["object_prim_path"]
    )
    setup2.target_marker_path = task2_config["target_marker_path"]
    robot2 = setup2.create_robot()
    setup2.create_object(mass=0.01)
    setup2.create_target_marker()

    controller2 = PickPlaceController(
        name="controller_task2",
        robot_articulation=robot2,
        gripper=robot2.gripper
    )
    articulation_controller2 = robot2.get_articulation_controller()

    print("✓ Scene setup complete")

    # ========================================================================
    # 3. Setup Scheduler with Task1 Disabled
    # ========================================================================
    print("\n→ Setting up scheduler...")
    scheduler = TaskScheduler()

    # Add Task1 - DISABLED
    scheduler.add_task(
        task_id="task1",
        setup=setup1,
        robot=robot1,
        controller=controller1,
        articulation_controller=articulation_controller1,
        robot_position=task1_config["robot_position"],
        enabled=True,
        order=0,
        pause_after=0.0
    )

    # Add Task2 - ENABLED
    scheduler.add_task(
        task_id="task2",
        setup=setup2,
        robot=robot2,
        controller=controller2,
        articulation_controller=articulation_controller2,
        robot_position=task2_config["robot_position"],
        enabled=True,
        order=0,
        pause_after=0.0
    )

    print("✓ Scheduler configured")

    # ========================================================================
    # 4. Physics Loop and Timeline Events
    # ========================================================================
    def simulation_step(step_size):
        """Called every physics step"""
        all_done = scheduler.step(delta_time=step_size)

        if all_done:
            scheduler.print_status()

    def on_timeline_event(event):
        """Handle timeline events (play/stop)"""
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            print("[Example1] Timeline stopped - resetting scheduler...")
            scheduler.reset()

    # Subscribe to physics updates
    physics_subscription = _physx.get_physx_interface().subscribe_physics_step_events(simulation_step)

    # Subscribe to timeline events
    timeline = get_timeline_interface()
    timeline_subscription = timeline.get_timeline_event_stream().create_subscription_to_pop(on_timeline_event)

    print("\n" + "="*70)
    print("READY - EXAMPLE 1")
    print("="*70)
    print("→ Task1: DISABLED (created but won't move)")
    print("→ Task2: ENABLED (will perform pick-and-place)")
    print("→ Press ▶ PLAY button to start")
    print("→ To stop: physics_subscription.unsubscribe()")
    print("="*70)

    return scheduler, physics_subscription, timeline_subscription


# ============================================================================
# EXAMPLE 2: Three Tasks in Sequence with Pauses
# ============================================================================
def example_2_sequential_with_pauses():
    """
    Example 2: Three tasks in sequence with pauses

    Demonstrates:
    - Creating three tasks
    - Sequential execution: task1 → pause 3s → task2 → pause 6s → task3
    - Configurable pause durations between tasks

    Expected behavior:
    - Task1 executes first
    - After completion, system pauses for 3 seconds
    - Task2 executes
    - After completion, system pauses for 6 seconds
    - Task3 executes
    - Simulation completes
    """
    print("="*70)
    print("EXAMPLE 2: THREE TASKS IN SEQUENCE WITH PAUSES")
    print("="*70)

    # ========================================================================
    # 1. Define Task Configurations
    # ========================================================================
    task1_config = {
        "robot_position": np.array([0.0, 0.0, 0.0]),
        "initial_position": np.array([-0.5, 0.4, 0.04]),
        "target_position": np.array([-0.6, -0.5, 0.04]),
        "custom_usd_path": "D:/poc/po_wiwynn_test/tst_cylinder01.usda",
        "robot_prim_path": "/World/cobotta_task1",
        "object_prim_path": "/World/pickup_object_task1",
        "target_marker_path": "/World/target_marker_task1"
    }

    task2_config = {
        "robot_position": np.array([1.0, 0.0, 0.0]),
        "initial_position": np.array([0.5, 0.4, 0.04]),
        "target_position": np.array([0.4, -0.5, 0.04]),
        "custom_usd_path": "D:/poc/po_wiwynn_test/tst_cylinder01.usda",
        "robot_prim_path": "/World/cobotta_task2",
        "object_prim_path": "/World/pickup_object_task2",
        "target_marker_path": "/World/target_marker_task2"
    }

    task3_config = {
        "robot_position": np.array([2.0, 0.0, 0.0]),
        "initial_position": np.array([1.5, 0.4, 0.04]),
        "target_position": np.array([1.4, -0.5, 0.04]),
        "custom_usd_path": "D:/poc/po_wiwynn_test/tst_cylinder01.usda",
        "robot_prim_path": "/World/cobotta_task3",
        "object_prim_path": "/World/pickup_object_task3",
        "target_marker_path": "/World/target_marker_task3"
    }

    # ========================================================================
    # 2. Setup Scene - Create All Three Tasks
    # ========================================================================
    print("\n→ Setting up scene with 3 tasks...")

    # Add table
    table_path = "D:/poc/po_wiwynn_test/prp_table02.usda"
    add_reference_to_stage(usd_path=table_path, prim_path="/World/Table")

    # Create Task 1
    print("  → Creating Task1...")
    setup1 = PickPlaceSceneSetup(
        custom_usd_path=task1_config["custom_usd_path"],
        object_initial_position=task1_config["initial_position"],
        target_position=task1_config["target_position"],
        robot_position=task1_config["robot_position"],
        robot_prim_path=task1_config["robot_prim_path"],
        object_prim_path=task1_config["object_prim_path"]
    )
    setup1.target_marker_path = task1_config["target_marker_path"]
    robot1 = setup1.create_robot()
    setup1.create_object(mass=0.01)
    setup1.create_target_marker()

    controller1 = PickPlaceController(
        name="controller_task1",
        robot_articulation=robot1,
        gripper=robot1.gripper
    )
    articulation_controller1 = robot1.get_articulation_controller()

    # Create Task 2
    print("  → Creating Task2...")
    setup2 = PickPlaceSceneSetup(
        custom_usd_path=task2_config["custom_usd_path"],
        object_initial_position=task2_config["initial_position"],
        target_position=task2_config["target_position"],
        robot_position=task2_config["robot_position"],
        robot_prim_path=task2_config["robot_prim_path"],
        object_prim_path=task2_config["object_prim_path"]
    )
    setup2.target_marker_path = task2_config["target_marker_path"]
    robot2 = setup2.create_robot()
    setup2.create_object(mass=0.01)
    setup2.create_target_marker()

    controller2 = PickPlaceController(
        name="controller_task2",
        robot_articulation=robot2,
        gripper=robot2.gripper
    )
    articulation_controller2 = robot2.get_articulation_controller()

    # Create Task 3
    print("  → Creating Task3...")
    setup3 = PickPlaceSceneSetup(
        custom_usd_path=task3_config["custom_usd_path"],
        object_initial_position=task3_config["initial_position"],
        target_position=task3_config["target_position"],
        robot_position=task3_config["robot_position"],
        robot_prim_path=task3_config["robot_prim_path"],
        object_prim_path=task3_config["object_prim_path"]
    )
    setup3.target_marker_path = task3_config["target_marker_path"]
    robot3 = setup3.create_robot()
    setup3.create_object(mass=0.01)
    setup3.create_target_marker()

    controller3 = PickPlaceController(
        name="controller_task3",
        robot_articulation=robot3,
        gripper=robot3.gripper
    )
    articulation_controller3 = robot3.get_articulation_controller()

    print("✓ Scene setup complete")

    # ========================================================================
    # 3. Setup Scheduler with Pauses
    # ========================================================================
    print("\n→ Setting up scheduler with pauses...")
    scheduler = TaskScheduler()

    # Add Task1 - Will pause 3 seconds after completion
    scheduler.add_task(
        task_id="task1",
        setup=setup1,
        robot=robot1,
        controller=controller1,
        articulation_controller=articulation_controller1,
        robot_position=task1_config["robot_position"],
        enabled=True,
        order=0,
        pause_after=3.0  # ⏸ Pause 3 seconds after completion
    )

    # Add Task2 - Will pause 6 seconds after completion
    scheduler.add_task(
        task_id="task2",
        setup=setup2,
        robot=robot2,
        controller=controller2,
        articulation_controller=articulation_controller2,
        robot_position=task2_config["robot_position"],
        enabled=True,
        order=0,
        pause_after=6.0  # ⏸ Pause 6 seconds after completion
    )

    # Add Task3 - No pause after (it's the last task)
    scheduler.add_task(
        task_id="task3",
        setup=setup3,
        robot=robot3,
        controller=controller3,
        articulation_controller=articulation_controller3,
        robot_position=task3_config["robot_position"],
        enabled=True,
        order=2,
        pause_after=0.0  # No pause (last task)
    )

    print("✓ Scheduler configured")

    # ========================================================================
    # 4. Physics Loop and Timeline Events
    # ========================================================================
    def simulation_step(step_size):
        """Called every physics step"""
        all_done = scheduler.step(delta_time=step_size)

        if all_done:
            scheduler.print_status()

    def on_timeline_event(event):
        """Handle timeline events (play/stop)"""
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            print("[Example2] Timeline stopped - resetting scheduler...")
            scheduler.reset()

    # Subscribe to physics updates
    physics_subscription = _physx.get_physx_interface().subscribe_physics_step_events(simulation_step)

    # Subscribe to timeline events
    timeline = get_timeline_interface()
    timeline_subscription = timeline.get_timeline_event_stream().create_subscription_to_pop(on_timeline_event)

    print("\n" + "="*70)
    print("READY - EXAMPLE 2")
    print("="*70)
    print("→ Task1: Will execute first, then pause 3 seconds")
    print("→ Task2: Will execute after pause, then pause 6 seconds")
    print("→ Task3: Will execute after pause (no pause after)")
    print("→ Press ▶ PLAY button to start")
    print("→ To stop: physics_subscription.unsubscribe()")
    print("="*70)

    return scheduler, physics_subscription, timeline_subscription


# ============================================================================
# Quick Run Examples
# ============================================================================
if __name__ == "__main__":
    print("\nAvailable examples:")
    print("1. example_1_disabled_task() - Two tasks with task1 disabled")
    print("2. example_2_sequential_with_pauses() - Three tasks with pauses")
    print("\nTo run an example, call the function:")
    print("  scheduler, physics_sub, timeline_sub = example_1_disabled_task()")
    print("  scheduler, physics_sub, timeline_sub = example_2_sequential_with_pauses()")

scheduler, physics_sub, timeline_sub = example_1_disabled_task()
#scheduler, physics_sub, timeline_sub = example_2_sequential_with_pauses()