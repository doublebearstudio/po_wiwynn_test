"""
Robot Pick-and-Place Extension for Existing USD Scene

This extension integrates robot pick-and-place functionality with an existing
USD scene file containing a Denso Cobotta robot and target cube.

Features:
- Loads existing USD scene with robot and cube
- Start button: Initiates pick-and-place sequence
- Stop button: Stops simulation and resets
- Automatic return to home position after completion

Scene Requirements:
- Robot prim: /World/cobotta_pro_900
- Cube prim: /World/geo_cube_01
- PhysicsScene must exist
- Cube must have physics properties

Usage:
1. Enable extension in Isaac Sim
2. Click "Start" to begin pick-and-place
3. Click "Stop" to halt and reset
"""

import omni.ext
import omni.ui as ui
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import ParallelGripper
import numpy as np
import asyncio
from pathlib import Path


class PickPlaceState:
    """State machine for pick-and-place operation."""
    IDLE = 0
    MOVING_TO_PICKUP = 1
    PICKING_UP = 2
    MOVING_TO_PLACE = 3
    PLACING = 4
    RETURNING_HOME = 5
    COMPLETE = 6


class RobotPickupExtension(omni.ext.IExt):
    """
    Extension for controlling robot pick-and-place in existing scene.

    Manages:
    - Scene loading
    - Robot gripper configuration
    - Pick-and-place state machine
    - UI controls (Start/Stop)
    """

    def on_startup(self, ext_id: str):
        """Initialize the extension and create UI."""
        print("[RobotPickupExtension] Extension startup")

        # ====================================================================
        # Extension State Variables
        # ====================================================================
        self._world = None
        self._robot = None
        self._gripper = None
        self._articulation_controller = None
        self._cube_prim = None
        self._is_running = False
        self._state = PickPlaceState.IDLE
        self._home_joint_positions = None

        # Scene configuration
        self._scene_path = r"D:\poc\po_wiwynn_test\po_wiwynn_test_v0003.usda"
        self._robot_prim_path = "/World/cobotta_pro_900"
        self._cube_prim_path = "/World/geo_cube_01"

        # Task parameters
        self._pickup_position = np.array([1.0, 0.0, 0.025])  # Current cube position
        self._place_position = np.array([0.0, 1.0, 0.025])   # Target drop position
        self._end_effector_offset = np.array([0, 0, 0.25])   # Gripper offset

        # ====================================================================
        # Create UI Window
        # ====================================================================
        self._window = ui.Window("Robot Pick & Place Control", width=400, height=300)

        with self._window.frame:
            with ui.VStack(spacing=10, height=0):
                # Title
                ui.Label(
                    "Denso Cobotta Pick & Place",
                    style={"font_size": 20},
                    alignment=ui.Alignment.CENTER
                )

                ui.Spacer(height=10)

                # Status display
                ui.Label("Status:", style={"font_size": 14})
                self._status_label = ui.Label(
                    "Ready - Click Start to begin",
                    style={"font_size": 12, "color": 0xFF00FF00}
                )

                ui.Spacer(height=10)

                # Scene info
                ui.Label("Scene Configuration:", style={"font_size": 14})
                with ui.VStack(spacing=5):
                    ui.Label(f"Scene: {Path(self._scene_path).name}", style={"font_size": 10})
                    ui.Label(f"Robot: {self._robot_prim_path}", style={"font_size": 10})
                    ui.Label(f"Cube: {self._cube_prim_path}", style={"font_size": 10})
                    ui.Label(f"Pickup: {self._pickup_position}", style={"font_size": 10})
                    ui.Label(f"Place: {self._place_position}", style={"font_size": 10})

                ui.Spacer(height=10)

                # Control buttons
                with ui.HStack(spacing=10):
                    self._start_button = ui.Button(
                        "Start",
                        clicked_fn=self._on_start_clicked,
                        height=40,
                        style={"Button": {"background_color": 0xFF00AA00}}
                    )
                    self._stop_button = ui.Button(
                        "Stop",
                        clicked_fn=self._on_stop_clicked,
                        height=40,
                        enabled=False,
                        style={"Button": {"background_color": 0xFFAA0000}}
                    )

                ui.Spacer(height=10)

                # Reset scene button
                self._reset_button = ui.Button(
                    "Reset Scene",
                    clicked_fn=self._on_reset_scene_clicked,
                    height=30
                )

    def on_shutdown(self):
        """Clean up extension resources."""
        print("[RobotPickupExtension] Extension shutdown")

        # Stop simulation if running
        if self._is_running:
            self._stop_simulation()

        # Clean up world
        if self._world:
            self._world.clear()
            self._world = None

        # Close window
        if self._window:
            self._window.destroy()
            self._window = None

    # ========================================================================
    # UI Callbacks
    # ========================================================================

    def _on_start_clicked(self):
        """Handle Start button click - Initialize and begin pick-and-place."""
        print("[RobotPickupExtension] Start button clicked")

        # Initialize scene if not already done
        if not self._world:
            if not self._initialize_scene():
                self._update_status("Error: Failed to initialize scene", error=True)
                return

        # Start the pick-and-place operation
        self._start_simulation()

    def _on_stop_clicked(self):
        """Handle Stop button click - Stop simulation and reset."""
        print("[RobotPickupExtension] Stop button clicked")
        self._stop_simulation()

    def _on_reset_scene_clicked(self):
        """Handle Reset Scene button - Reload scene from file."""
        print("[RobotPickupExtension] Reset scene button clicked")

        # Stop if running
        if self._is_running:
            self._stop_simulation()

        # Clear world
        if self._world:
            self._world.clear()
            self._world = None

        # Reload scene
        self._initialize_scene()
        self._update_status("Scene reset complete", error=False)

    # ========================================================================
    # Scene Initialization
    # ========================================================================

    def _initialize_scene(self) -> bool:
        """
        Initialize the scene from USD file and configure robot.

        Returns:
            bool: True if initialization successful, False otherwise
        """
        try:
            self._update_status("Loading scene...", error=False)

            # ----------------------------------------------------------------
            # 1. Load USD Scene
            # ----------------------------------------------------------------
            print(f"[RobotPickupExtension] Loading scene: {self._scene_path}")

            if not Path(self._scene_path).exists():
                print(f"[RobotPickupExtension] ERROR: Scene file not found: {self._scene_path}")
                return False

            # Open the USD stage
            open_stage(self._scene_path)

            # ----------------------------------------------------------------
            # 2. Create World (Physics already exists in scene)
            # ----------------------------------------------------------------
            self._update_status("Creating world...", error=False)

            # Create world but don't add physics scene (already in USD)
            self._world = World(stage_units_in_meters=1.0)
            await self._world.initialize_simulation_context_async()

            # ----------------------------------------------------------------
            # 3. Configure Robot with Gripper
            # ----------------------------------------------------------------
            self._update_status("Configuring robot...", error=False)

            if not self._setup_robot():
                return False

            # ----------------------------------------------------------------
            # 4. Get Cube Reference
            # ----------------------------------------------------------------
            self._update_status("Finding cube...", error=False)

            from omni.isaac.core.utils.stage import get_current_stage
            stage = get_current_stage()
            self._cube_prim = stage.GetPrimAtPath(self._cube_prim_path)

            if not self._cube_prim.IsValid():
                print(f"[RobotPickupExtension] ERROR: Cube not found at {self._cube_prim_path}")
                return False

            # ----------------------------------------------------------------
            # 5. Reset World
            # ----------------------------------------------------------------
            self._update_status("Resetting world...", error=False)
            await self._world.reset_async()

            # Store home position
            self._home_joint_positions = self._robot.get_joint_positions()

            self._update_status("Initialization complete - Ready to start", error=False)
            print("[RobotPickupExtension] Scene initialization complete")

            return True

        except Exception as e:
            print(f"[RobotPickupExtension] ERROR during initialization: {e}")
            import traceback
            traceback.print_exc()
            return False

    def _setup_robot(self) -> bool:
        """
        Configure the robot with gripper from existing scene.

        Returns:
            bool: True if setup successful, False otherwise
        """
        try:
            # ----------------------------------------------------------------
            # Configure Gripper
            # ----------------------------------------------------------------
            gripper = ParallelGripper(
                end_effector_prim_path=f"{self._robot_prim_path}/onrobot_rg6_base_link",
                joint_prim_names=["finger_joint", "right_outer_knuckle_joint"],
                joint_opened_positions=np.array([0, 0]),
                joint_closed_positions=np.array([0.628, -0.628]),
                action_deltas=np.array([-0.2, 0.2])
            )

            # ----------------------------------------------------------------
            # Create Manipulator Wrapper
            # ----------------------------------------------------------------
            self._robot = SingleManipulator(
                prim_path=self._robot_prim_path,
                name="cobotta_robot",
                end_effector_prim_name="onrobot_rg6_base_link",
                gripper=gripper
            )

            # Add to world scene
            self._world.scene.add(self._robot)

            # Initialize gripper
            gripper.initialize()
            self._gripper = gripper

            # Get articulation controller
            self._articulation_controller = self._robot.get_articulation_controller()

            print("[RobotPickupExtension] Robot configuration complete")
            return True

        except Exception as e:
            print(f"[RobotPickupExtension] ERROR configuring robot: {e}")
            import traceback
            traceback.print_exc()
            return False

    # ========================================================================
    # Simulation Control
    # ========================================================================

    def _start_simulation(self):
        """Start the pick-and-place simulation."""
        self._is_running = True
        self._state = PickPlaceState.MOVING_TO_PICKUP

        # Update UI
        self._start_button.enabled = False
        self._stop_button.enabled = True
        self._update_status("Running: Moving to pickup position...", error=False)

        # Start simulation loop
        asyncio.ensure_future(self._simulation_loop())

    def _stop_simulation(self):
        """Stop the simulation and reset state."""
        self._is_running = False
        self._state = PickPlaceState.IDLE

        # Update UI
        self._start_button.enabled = True
        self._stop_button.enabled = False
        self._update_status("Stopped", error=False)

        # Stop physics
        if self._world:
            self._world.pause()

    async def _simulation_loop(self):
        """
        Main simulation loop - runs pick-and-place state machine.

        State sequence:
        1. MOVING_TO_PICKUP: Move above cube
        2. PICKING_UP: Lower, grasp, lift
        3. MOVING_TO_PLACE: Transport to target
        4. PLACING: Lower, release, retreat
        5. RETURNING_HOME: Return to home position
        6. COMPLETE: Task done
        """
        while self._is_running and self._world:
            # Step simulation
            await self._world.play_async()

            # Execute current state
            if self._state == PickPlaceState.MOVING_TO_PICKUP:
                self._handle_move_to_pickup()

            elif self._state == PickPlaceState.PICKING_UP:
                self._handle_picking_up()

            elif self._state == PickPlaceState.MOVING_TO_PLACE:
                self._handle_move_to_place()

            elif self._state == PickPlaceState.PLACING:
                self._handle_placing()

            elif self._state == PickPlaceState.RETURNING_HOME:
                self._handle_returning_home()

            elif self._state == PickPlaceState.COMPLETE:
                self._update_status("Complete! Click Start to repeat or Stop to halt.", error=False)
                self._is_running = False
                self._start_button.enabled = True
                self._stop_button.enabled = False
                break

            # Small delay to prevent overwhelming the system
            await asyncio.sleep(0.01)

    # ========================================================================
    # State Handlers (Simplified - Full implementation would use controller)
    # ========================================================================

    def _handle_move_to_pickup(self):
        """Move robot to pickup position."""
        # TODO: Implement IK or controller to move to pickup position
        # For now, transition to next state
        self._update_status("Picking up cube...", error=False)
        self._state = PickPlaceState.PICKING_UP

    def _handle_picking_up(self):
        """Execute pickup: lower, grasp, lift."""
        # TODO: Implement gripper control
        # Close gripper
        # self._gripper.close()

        self._update_status("Moving to place position...", error=False)
        self._state = PickPlaceState.MOVING_TO_PLACE

    def _handle_move_to_place(self):
        """Move robot to place position."""
        # TODO: Implement IK or controller to move to place position
        self._update_status("Placing cube...", error=False)
        self._state = PickPlaceState.PLACING

    def _handle_placing(self):
        """Execute placing: lower, release, retreat."""
        # TODO: Implement gripper control
        # Open gripper
        # self._gripper.open()

        self._update_status("Returning to home position...", error=False)
        self._state = PickPlaceState.RETURNING_HOME

    def _handle_returning_home(self):
        """Return robot to home position."""
        # TODO: Implement return to home
        # Move robot back to stored home joint positions

        self._state = PickPlaceState.COMPLETE

    # ========================================================================
    # UI Helpers
    # ========================================================================

    def _update_status(self, message: str, error: bool = False):
        """
        Update status label with message.

        Args:
            message: Status message to display
            error: If True, display in red. Otherwise green.
        """
        if self._status_label:
            self._status_label.text = message
            color = 0xFFFF0000 if error else 0xFF00FF00
            self._status_label.style = {"font_size": 12, "color": color}

        print(f"[RobotPickupExtension] Status: {message}")
