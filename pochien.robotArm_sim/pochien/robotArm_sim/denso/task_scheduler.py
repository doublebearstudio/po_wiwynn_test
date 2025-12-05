"""
Task Scheduler for Pick and Place Operations

This module provides a scheduler for managing multiple pick-and-place tasks with
advanced event controls including:
- Task ordering (sequential execution)
- Enable/disable individual tasks
- Configurable pauses between tasks
- Task state management

Usage:
    from task_scheduler import TaskScheduler, ScheduledTask

    # Create scheduler
    scheduler = TaskScheduler()

    # Add tasks
    scheduler.add_task(
        task_id="task1",
        setup=setup1,
        robot=robot1,
        controller=controller1,
        articulation_controller=articulation_controller1,
        robot_position=np.array([0.0, 0.0, 0.0]),
        enabled=True,
        pause_after=3.0  # Pause 3 seconds after completion
    )

    # In simulation loop
    scheduler.step()

Author: Generated for Wiwynn Test
Date: 2025-12-03
"""

import numpy as np
from typing import Dict, List, Optional, Any
from enum import Enum
import time


class TaskState(Enum):
    """Task execution states"""
    PENDING = "pending"           # Not started yet
    RUNNING = "running"           # Currently executing
    PAUSING = "pausing"          # Waiting between tasks
    COMPLETED = "completed"       # Successfully finished
    DISABLED = "disabled"         # Disabled by user
    ERROR = "error"              # Encountered an error


class ScheduledTask:
    """
    Represents a single scheduled pick-and-place task.

    Attributes:
        task_id: Unique identifier for the task
        setup: PickPlaceSceneSetup instance
        robot: Robot manipulator instance
        controller: PickPlaceController instance
        articulation_controller: Articulation controller for the robot
        robot_position: Robot base position in world coordinates
        enabled: Whether the task is enabled
        order: Execution order (0 = first, 1 = second, etc.)
        pause_after: Seconds to pause after completion before next task
        state: Current execution state
    """

    def __init__(
        self,
        task_id: str,
        setup: Any,
        robot: Any,
        controller: Any,
        articulation_controller: Any,
        robot_position: np.ndarray,
        enabled: bool = True,
        order: int = 0,
        pause_after: float = 0.0
    ):
        self.task_id = task_id
        self.setup = setup
        self.robot = robot
        self.controller = controller
        self.articulation_controller = articulation_controller
        self.robot_position = robot_position
        self.enabled = enabled
        self.order = order
        self.pause_after = pause_after

        # State tracking
        self.state = TaskState.DISABLED if not enabled else TaskState.PENDING
        self.initialized = False
        self.completion_logged = False

        # Pause timing (individual task, not used for group pause)
        self.pause_start_time = None
        self.pause_elapsed = 0.0

    def reset(self):
        """Reset task to initial state for replay"""
        self.state = TaskState.DISABLED if not self.enabled else TaskState.PENDING
        self.initialized = False
        self.completion_logged = False
        self.pause_start_time = None
        self.pause_elapsed = 0.0
        if hasattr(self, '_debug_printed'):
            delattr(self, '_debug_printed')  # Clear debug flag for next run
        if self.controller:
            self.controller.reset()

    def __repr__(self):
        return f"ScheduledTask(id={self.task_id}, order={self.order}, state={self.state.value}, enabled={self.enabled})"


class TaskScheduler:
    """
    Manages and schedules multiple pick-and-place tasks.

    Features:
    - Sequential task execution based on order
    - Enable/disable individual tasks
    - Configurable pauses between tasks
    - State management and logging

    Usage:
        scheduler = TaskScheduler()
        scheduler.add_task(...)

        # In simulation loop:
        scheduler.step()
    """

    def __init__(self):
        self.tasks: Dict[str, ScheduledTask] = {}
        self.task_order: List[str] = []  # Ordered list of task IDs
        self.order_groups: List[List[str]] = []  # Groups of task IDs by order value
        self.current_order_group_index: int = 0
        self.all_initialized = False
        self.simulation_time = 0.0
        self.group_pause_start_time: Optional[float] = None  # Track pause for entire group
        self.group_pause_duration: float = 0.0

    def add_task(
        self,
        task_id: str,
        setup: Any,
        robot: Any,
        controller: Any,
        articulation_controller: Any,
        robot_position: np.ndarray,
        enabled: bool = True,
        order: Optional[int] = None,
        pause_after: float = 0.0
    ) -> ScheduledTask:
        """
        Add a task to the scheduler.

        Args:
            task_id: Unique identifier for the task
            setup: PickPlaceSceneSetup instance
            robot: Robot manipulator instance
            controller: PickPlaceController instance
            articulation_controller: Articulation controller
            robot_position: Robot base position [x, y, z]
            enabled: Whether task is enabled (default: True)
            order: Execution order (if None, appends to end)
            pause_after: Seconds to pause after completion (default: 0.0)

        Returns:
            The created ScheduledTask instance
        """
        if task_id in self.tasks:
            raise ValueError(f"Task '{task_id}' already exists in scheduler")

        # Determine order
        if order is None:
            order = len(self.tasks)

        # DEBUG: Print input robot_position
        print(f"[DEBUG] Adding '{task_id}':")
        print(f"  robot_position = {robot_position}, id = {id(robot_position)}")
        print(f"  setup.object_prim_path = {setup.object_prim_path}")
        print(f"  setup.target_position = {setup.target_position}")
        print(f"  controller = {controller.name if hasattr(controller, 'name') else controller}, id = {id(controller)}")
        print(f"  articulation_controller id = {id(articulation_controller)}")
        print(f"  robot = {robot.name if hasattr(robot, 'name') else robot}, id = {id(robot)}")

        task = ScheduledTask(
            task_id=task_id,
            setup=setup,
            robot=robot,
            controller=controller,
            articulation_controller=articulation_controller,
            robot_position=robot_position,
            enabled=enabled,
            order=order,
            pause_after=pause_after
        )

        # DEBUG: Print stored values
        print(f"[DEBUG] Stored '{task_id}': task.robot_position = {task.robot_position}, id = {id(task.robot_position)}")

        self.tasks[task_id] = task
        self._rebuild_task_order()

        print(f"[TaskScheduler] Added task '{task_id}' (order={order}, enabled={enabled}, pause={pause_after}s)")
        return task

    def remove_task(self, task_id: str):
        """Remove a task from the scheduler"""
        if task_id in self.tasks:
            del self.tasks[task_id]
            self._rebuild_task_order()
            print(f"[TaskScheduler] Removed task '{task_id}'")

    def enable_task(self, task_id: str):
        """Enable a task"""
        if task_id in self.tasks:
            self.tasks[task_id].enabled = True
            if self.tasks[task_id].state == TaskState.DISABLED:
                self.tasks[task_id].state = TaskState.PENDING
            print(f"[TaskScheduler] Enabled task '{task_id}'")

    def disable_task(self, task_id: str):
        """Disable a task"""
        if task_id in self.tasks:
            self.tasks[task_id].enabled = False
            self.tasks[task_id].state = TaskState.DISABLED
            print(f"[TaskScheduler] Disabled task '{task_id}'")

    def set_task_order(self, task_id: str, new_order: int):
        """Change the execution order of a task"""
        if task_id in self.tasks:
            self.tasks[task_id].order = new_order
            self._rebuild_task_order()
            print(f"[TaskScheduler] Task '{task_id}' order changed to {new_order}")

    def set_pause_after(self, task_id: str, pause_seconds: float):
        """Set pause duration after task completion"""
        if task_id in self.tasks:
            self.tasks[task_id].pause_after = pause_seconds
            print(f"[TaskScheduler] Task '{task_id}' pause_after set to {pause_seconds}s")

    def _rebuild_task_order(self):
        """
        Rebuild the ordered list and groups of task IDs based on order attribute.

        Tasks with the same order value are grouped together and execute in parallel.
        Groups are ordered by their order value (ascending).
        """
        # Sort all task IDs by order value
        self.task_order = sorted(
            self.tasks.keys(),
            key=lambda tid: self.tasks[tid].order
        )

        # Group tasks by their order value for parallel execution
        self.order_groups = []
        if not self.task_order:
            return

        # Group consecutive tasks with the same order value
        from itertools import groupby
        for order_value, task_ids in groupby(self.task_order, key=lambda tid: self.tasks[tid].order):
            self.order_groups.append(list(task_ids))

    def initialize_all(self):
        """Initialize all robots and controllers (call once at start)"""
        if self.all_initialized:
            return True

        # Try to initialize all tasks
        all_success = True
        for task_id in self.task_order:
            task = self.tasks[task_id]

            # Skip already initialized tasks
            if task.initialized:
                continue

            # Skip disabled tasks
            if not task.enabled:
                continue

            try:
                task.robot.initialize()
                task.controller.reset()
                task.initialized = True
                print(f"[TaskScheduler] ✓ Initialized task '{task_id}'")
            except Exception as e:
                # Physics context might not be ready yet - retry on next frame
                # Don't mark as ERROR, just return False to retry later
                all_success = False
                # Silently return - will retry next frame
                return False

        # All tasks successfully initialized
        if all_success:
            self.all_initialized = True
            self.current_order_group_index = 0
            print(f"[TaskScheduler] All {len(self.tasks)} tasks initialized successfully")
            return True

        return False

    def reset(self):
        """Reset all tasks for replay"""
        print("[TaskScheduler] Resetting all tasks...")
        for task in self.tasks.values():
            task.reset()
        self.all_initialized = False
        self.current_order_group_index = 0
        self.simulation_time = 0.0
        self.group_pause_start_time = None
        self.group_pause_duration = 0.0
        print("[TaskScheduler] Reset complete")

    def step(self, delta_time: float = 0.016) -> bool:
        """
        Execute one scheduler step (call every physics frame).

        Tasks with the same order value execute in parallel.

        Args:
            delta_time: Time elapsed since last step (seconds)

        Returns:
            True if all tasks completed, False otherwise
        """
        self.simulation_time += delta_time

        # Initialize if not done yet
        if not self.all_initialized:
            # Try to initialize, return False if not ready yet
            if not self.initialize_all():
                return False  # Not initialized yet, will retry next frame

        # Check if all order groups completed
        if self.current_order_group_index >= len(self.order_groups):
            return True  # All done

        # Handle group-level pause
        if self.group_pause_start_time is not None:
            elapsed = self.simulation_time - self.group_pause_start_time
            if elapsed >= self.group_pause_duration:
                print(f"[TaskScheduler] ⏭ Group pause complete, moving to next order group...")
                self.group_pause_start_time = None
                self.group_pause_duration = 0.0
                self.current_order_group_index += 1
                return False
            else:
                return False  # Still pausing

        # Get current order group (list of task IDs)
        current_group = self.order_groups[self.current_order_group_index]

        # Track if any task started in this frame (for logging)
        group_started = False

        # Execute all tasks in the current group (parallel execution)
        for task_id in current_group:
            task = self.tasks[task_id]

            # Skip disabled tasks
            if not task.enabled:
                task.state = TaskState.DISABLED
                continue

            # Handle task states
            if task.state == TaskState.PENDING:
                # Start the task
                task.state = TaskState.RUNNING
                if not group_started:
                    print(f"[TaskScheduler] ▶ Starting order group {task.order} (parallel tasks: {', '.join(current_group)})")
                    group_started = True

            elif task.state == TaskState.RUNNING:
                # Execute task
                self._execute_task(task)

                # Check if completed
                if task.controller.is_done():
                    if not task.completion_logged:
                        print(f"[TaskScheduler] ✓ Task '{task_id}' completed")
                        task.completion_logged = True
                        task.state = TaskState.COMPLETED

        # Check if ALL tasks in the group are completed or disabled
        all_done = True
        for task_id in current_group:
            task = self.tasks[task_id]
            if task.enabled and task.state not in [TaskState.COMPLETED, TaskState.ERROR]:
                all_done = False
                break

        # If all tasks in group are done, check for pause
        if all_done:
            # Find maximum pause_after from all tasks in this group
            max_pause = 0.0
            for task_id in current_group:
                task = self.tasks[task_id]
                if task.enabled and task.pause_after > max_pause:
                    max_pause = task.pause_after

            if max_pause > 0:
                print(f"[TaskScheduler] ⏸ Order group {self.tasks[current_group[0]].order} completed, pausing for {max_pause}s...")
                self.group_pause_start_time = self.simulation_time
                self.group_pause_duration = max_pause
            else:
                # No pause, move to next group immediately
                print(f"[TaskScheduler] ✓ Order group {self.tasks[current_group[0]].order} completed")
                self.current_order_group_index += 1

        return False  # Not all done yet

    def _execute_task(self, task: ScheduledTask):
        """Execute a single task step"""
        try:
            # Get observations
            observations = task.setup.get_observations(task.robot)

            # Get world coordinates - controller expects world coordinates, not robot-local
            object_world_pos = observations["pickup_object"]["position"]
            target_world_pos = observations["pickup_object"]["target_position"]

            # DEBUG: Print coordinate information (only once per task start)
            if not hasattr(task, '_debug_printed'):
                print(f"\n[DEBUG] Executing '{task.task_id}':")
                print(f"  setup.object_prim_path = {task.setup.object_prim_path}")
                print(f"  setup.target_position = {task.setup.target_position}")
                print(f"  robot_position = {task.robot_position}, id = {id(task.robot_position)}")
                print(f"  object_world_pos = {object_world_pos}")
                print(f"  target_world_pos = {target_world_pos}")
                print(f"  controller = {task.controller.name if hasattr(task.controller, 'name') else task.controller}, id = {id(task.controller)}")
                print(f"  articulation_controller id = {id(task.articulation_controller)}")
                print(f"  robot = {task.robot.name if hasattr(task.robot, 'name') else task.robot}, id = {id(task.robot)}")
                print(f"  Passing world coordinates directly to controller (no transformation)")
                task._debug_printed = True

            # Use world positions directly - the controller/RMPFlow handles robot position internally
            object_local_pos = object_world_pos
            target_local_pos = target_world_pos

            # Compute actions
            actions = task.controller.forward(
                picking_position=object_local_pos,
                placing_position=target_local_pos,
                current_joint_positions=observations["cobotta_robot"]["joint_positions"],
                end_effector_offset=np.array([0, 0, 0.25]),
            )

            # Apply actions
            if actions:
                task.articulation_controller.apply_action(actions)

        except Exception as e:
            print(f"[TaskScheduler] Error executing task '{task.task_id}': {e}")
            task.state = TaskState.ERROR

    def get_status(self) -> Dict[str, Any]:
        """Get current scheduler status"""
        current_group = None
        if self.current_order_group_index < len(self.order_groups):
            current_group = self.order_groups[self.current_order_group_index]

        return {
            "total_tasks": len(self.tasks),
            "total_order_groups": len(self.order_groups),
            "current_order_group_index": self.current_order_group_index,
            "current_group_tasks": current_group,
            "tasks": {
                tid: {
                    "order": task.order,
                    "enabled": task.enabled,
                    "state": task.state.value,
                    "pause_after": task.pause_after
                }
                for tid, task in self.tasks.items()
            }
        }

    def print_status(self):
        """Print current scheduler status"""
        print("\n" + "="*70)
        print("TASK SCHEDULER STATUS (PARALLEL EXECUTION)")
        print("="*70)
        print(f"Total tasks: {len(self.tasks)}")
        print(f"Total order groups: {len(self.order_groups)}")
        print(f"Current order group: {self.current_order_group_index}/{len(self.order_groups)}")
        print(f"Simulation time: {self.simulation_time:.2f}s")
        print("\nOrder Groups:")
        for i, group in enumerate(self.order_groups):
            group_status = "→" if i == self.current_order_group_index else "✓" if i < self.current_order_group_index else "○"
            order_val = self.tasks[group[0]].order
            print(f"  {group_status} Group {i} (order={order_val}): {', '.join(group)}")
        print("\nTasks:")
        for task_id in self.task_order:
            task = self.tasks[task_id]
            status = "✓" if task.state == TaskState.COMPLETED else "▶" if task.state == TaskState.RUNNING else "⏸" if task.state == TaskState.PAUSING else "○"
            enabled_str = "ENABLED" if task.enabled else "DISABLED"
            print(f"  {status} [{task.order}] {task_id:15s} - {task.state.value:12s} ({enabled_str}, pause={task.pause_after}s)")
        print("="*70)
