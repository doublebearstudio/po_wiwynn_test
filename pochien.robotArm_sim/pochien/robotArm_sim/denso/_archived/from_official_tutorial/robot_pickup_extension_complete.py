"""
Complete Robot Pick-and-Place Extension for Existing USD Scene

This extension integrates full robot pick-and-place functionality with an existing
USD scene file. Uses the PickPlaceController for smooth, reliable motion.

Features:
- Loads existing USD scene (po_wiwynn_test_v0003.usda)
- Configures robot gripper automatically
- Start button: Pick cube from (1,0,0.025) and place at (0,1,0.025)
- Stop button: Stops simulation immediately
- Auto return to home position after completion

Author: Generated for Wiwynn Test
Date: 2025-11-26
"""

import omni.ext
import omni.ui as ui
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage, get_current_stage
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import ParallelGripper
import numpy as np
import asyncio
from pathlib import Path
import sys
import os

# Add denso directory to path for imports
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

# Import pick_place_controller from local denso directory
try:
    from pick_place_controller import PickPlaceController
except ImportError as e:
    print(f"Warning: Could not import PickPlaceController: {e}")
    PickPlaceController = None


class RobotPickupExtensionComplete(omni.ext.IExt):
    """
    Complete extension for robot pick-and-place in existing scene.

    This extension provides full integration with:
    - Existing USD scene loading
    - Robot gripper configuration    - PickPlaceController for motion control
    - Start/Stop UI controls
    - Automatic home position return
    """

    def on_startup(self, ext_id: str):
        """Initialize the extension and create UI."""
        print("[RobotPickup] Extension startup")

        # ====================================================================
        # State Variables
        # ====================================================================
        self._world = None
        self._robot = None
        self._gripper = None
        self._controller = None
        self._articulation_controller = None
        self._is_running = False
        self._task_complete = False
        self._subscription = None

        # Scene paths
        self._scene_path = r"D:\poc\po_wiwynn_test\po_wiwynn_test_v0003.usda"
        self._robot_prim_path = "/World/cobotta_pro_900"
        self._cube_prim_path = "/World/geo_cube_01"

        # Task configuration
        self._pickup_position = np.array([1.0, 0.0, 0.025])
        self._place_position = np.array([0.0, 1.0, 0.025])
        self._end_effector_offset = np.array([0, 0, 0.25])
        self._home_joint_positions = None

        # ====================================================================
        # Create UI
        # ====================================================================
        self._build_ui()

    def _build_ui(self):
        """Build the UI window with Start/Stop controls."""
        self._window = ui.Window("Robot Pick & Place Control", width=450, height=400)

        with self._window.frame:
            with ui.VStack(spacing=15, height=0):
                # ============================================================
                # Header
                # ============================================================
                ui.Label(
                    "Denso Cobotta Pick & Place Control",
                    style={"font_size": 22, "color": 0xFFFFFFFF},
                    alignment=ui.Alignment.CENTER,
                    height=30
                )

                ui.Separator()

                # ============================================================
                # Status Section
                # ============================================================
                with ui.CollapsableFrame("Status", height=0, collapsed=False):
                    with ui.VStack(spacing=5):
                        self._status_label = ui.Label(
                            "Ready - Click 'Start' to begin",
                            word_wrap=True,
                            style={"font_size": 13, "color": 0xFF00DD00}
                        )

                        # Progress indicator (optional future enhancement)
                        # self._progress = ui.ProgressBar(height=20)

                # ============================================================
                # Scene Configuration Section
                # ============================================================
                with ui.CollapsableFrame("Scene Configuration", height=0, collapsed=True):
                    with ui.VStack(spacing=3):
                        ui.Label(f"Scene File:", style={"font_size": 11})
                        ui.Label(
                            f"  {Path(self._scene_path).name}",
                            style={"font_size": 10, "color": 0xFFAAAAAA}
                        )

                        ui.Spacer(height=5)

                        ui.Label(f"Robot Prim:", style={"font_size": 11})
                        ui.Label(
                            f"  {self._robot_prim_path}",
                            style={"font_size": 10, "color": 0xFFAAAAAA}
                        )

                        ui.Spacer(height=5)

                        ui.Label(f"Target Cube:", style={"font_size": 11})
                        ui.Label(
                            f"  {self._cube_prim_path}",
                            style={"font_size": 10, "color": 0xFFAAAAAA}
                        )

                # ============================================================
                # Task Parameters Section
                # ============================================================
                with ui.CollapsableFrame("Task Parameters", height=0, collapsed=True):
                    with ui.VStack(spacing=3):
                        ui.Label("Pickup Position (m):", style={"font_size": 11})
                        ui.Label(
                            f"  X: {self._pickup_position[0]:.3f}, "
                            f"Y: {self._pickup_position[1]:.3f}, "
                            f"Z: {self._pickup_position[2]:.3f}",
                            style={"font_size": 10, "color": 0xFFAAAAAA}
                        )

                        ui.Spacer(height=5)

                        ui.Label("Place Position (m):", style={"font_size": 11})
                        ui.Label(
                            f"  X: {self._place_position[0]:.3f}, "
                            f"Y: {self._place_position[1]:.3f}, "
                            f"Z: {self._place_position[2]:.3f}",
                            style={"font_size": 10, "color": 0xFFAAAAAA}
                        )

                ui.Spacer(height=10)

                # ============================================================
                # Control Buttons
                # ============================================================
                ui.Label("Controls:", style={"font_size": 14})

                with ui.HStack(spacing=10):
                    self._start_button = ui.Button(
                        "Start Pick & Place",
                        clicked_fn=self._on_start_clicked,
                        height=50,
                        style={
                            "Button": {
                                "background_color": 0xFF228822,
                                "border_radius": 5,
                                "font_size": 16
                            }
                        }
                    )

                    self._stop_button = ui.Button(
                        "Stop",
                        clicked_fn=self._on_stop_clicked,
                        height=50,
                        enabled=False,
                        style={
                            "Button": {
                                "background_color": 0xFF882222,
                                "border_radius": 5,
                                "font_size": 16
                            }
                        }
                    )

                ui.Spacer(height=10)

                # ============================================================
                # Utility Buttons
                # ============================================================
                with ui.HStack(spacing=10):
                    ui.Button(
                        "Reset Scene",
                        clicked_fn=self._on_reset_scene_clicked,
                        height=35,
                        style={"Button": {"background_color": 0xFF555555}}
                    )

                    ui.Button(
                        "Return to Home",
                        clicked_fn=self._on_return_home_clicked,
                        height=35,
                        style={"Button": {"background_color": 0xFF555555}}
                    )

    def on_shutdown(self):
        """Clean up extension resources."""
        print("[RobotPickup] Extension shutdown")

        # Stop simulation
        if self._is_running:
            self._stop_simulation()

        # Unsubscribe from updates
        if self._subscription:
            self._subscription.unsubscribe()
            self._subscription = None

        # Clear world
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
        """Handle Start button - Initialize scene and begin pick-and-place."""
        print("[RobotPickup] Start button clicked")

        # Initialize if not done
        if not self._world:
            asyncio.ensure_future(self._initialize_scene_async())
        else:
            self._start_simulation()

    def _on_stop_clicked(self):
        """Handle Stop button - Stop simulation immediately."""
        print("[RobotPickup] Stop button clicked")
        self._stop_simulation()

    def _on_reset_scene_clicked(self):
        """Handle Reset Scene button - Reload scene from USD file."""
        print("[RobotPickup] Reset scene clicked")

        if self._is_running:
            self._stop_simulation()

        asyncio.ensure_future(self._reset_scene_async())

    def _on_return_home_clicked(self):
        """Handle Return Home button - Move robot to home position."""
        print("[RobotPickup] Return home clicked")

        if not self._robot or not self._home_joint_positions:
            self._update_status("Error: Robot not initialized", error=True)
            return

        # Set robot to home positions
        self._robot.set_joint_positions(self._home_joint_positions)
        self._update_status("Returned to home position", error=False)

    # ========================================================================
    # Scene Initialization
    # ========================================================================

    async def _initialize_scene_async(self):
        """Initialize scene asynchronously."""
        try:
            self._update_status("Loading scene file...", error=False)

            # Check file exists
            if not Path(self._scene_path).exists():
                self._update_status(f"Error: Scene file not found!", error=True)
                return

            # Load USD stage
            print(f"[RobotPickup] Loading: {self._scene_path}")
            open_stage(self._scene_path)

            await asyncio.sleep(0.5)  # Let stage load

            # Create World (don't add physics - already in USD)
            self._update_status("Creating simulation world...", error=False)
            self._world = World(stage_units_in_meters=1.0, add_ground_plane=False)

            # Configure robot
            self._update_status("Configuring robot...", error=False)
            if not await self._setup_robot_async():
                self._update_status("Error: Robot setup failed!", error=True)
                return

            # Reset world
            self._update_status("Initializing physics...", error=False)
            await self._world.reset_async()

            # Store home position
            self._home_joint_positions = self._robot.get_joint_positions()
            print(f"[RobotPickup] Home position: {self._home_joint_positions}")

            self._update_status("✓ Ready - Click Start to begin", error=False)

            # Auto-start
            await asyncio.sleep(1.0)
            self._start_simulation()

        except Exception as e:
            self._update_status(f"Error: {str(e)}", error=True)
            print(f"[RobotPickup] Initialization error: {e}")
            import traceback
            traceback.print_exc()

    async def _setup_robot_async(self) -> bool:
        """Configure robot with gripper."""
        try:
            # Configure gripper
            self._gripper = ParallelGripper(
                end_effector_prim_path=f"{self._robot_prim_path}/onrobot_rg6_base_link",
                joint_prim_names=["finger_joint", "right_outer_knuckle_joint"],
                joint_opened_positions=np.array([0, 0]),
                joint_closed_positions=np.array([0.628, -0.628]),
                action_deltas=np.array([-0.2, 0.2])
            )

            # Create robot manipulator
            self._robot = SingleManipulator(
                prim_path=self._robot_prim_path,
                name="cobotta_robot",
                end_effector_prim_name="onrobot_rg6_base_link",
                gripper=self._gripper
            )

            # Add to scene
            self._world.scene.add(self._robot)

            # Initialize gripper
            await asyncio.sleep(0.2)
            self._gripper.initialize()

            # Get articulation controller
            self._articulation_controller = self._robot.get_articulation_controller()

            # Create pick-place controller if available
            if PickPlaceController:
                self._controller = PickPlaceController(
                    name="controller",
                    robot_articulation=self._robot,
                    gripper=self._gripper
                )
                print("[RobotPickup] PickPlaceController initialized")
            else:
                print("[RobotPickup] Warning: PickPlaceController not available")

            return True

        except Exception as e:
            print(f"[RobotPickup] Robot setup error: {e}")
            import traceback
            traceback.print_exc()
            return False

    async def _reset_scene_async(self):
        """Reset scene asynchronously."""
        self._update_status("Resetting scene...", error=False)

        # Clear world
        if self._world:
            self._world.clear()
            self._world = None

        # Reinitialize
        await self._initialize_scene_async()

    # ========================================================================
    # Simulation Control
    # ========================================================================

    def _start_simulation(self):
        """Start the pick-and-place simulation."""
        if not self._robot or not self._controller:
            self._update_status("Error: System not initialized!", error=True)
            return

        self._is_running = True
        self._task_complete = False

        # Reset controller
        self._controller.reset()

        # Update UI
        self._start_button.enabled = False
        self._stop_button.enabled = True
        self._update_status("Running: Pick and place in progress...", error=False)

        # Subscribe to physics updates
        from omni.isaac.core.utils.extensions import get_extension_path_from_name
        self._subscription = (
            omni.physx.get_physx_interface()
            .subscribe_physics_step_events(self._on_physics_step)
        )

        # Start physics
        self._world.play()

        print("[RobotPickup] Simulation started")

    def _stop_simulation(self):
        """Stop the simulation."""
        self._is_running = False

        # Unsubscribe
        if self._subscription:
            self._subscription.unsubscribe()
            self._subscription = None

        # Pause physics
        if self._world:
            self._world.pause()

        # Update UI
        self._start_button.enabled = True
        self._stop_button.enabled = False
        self._update_status("Stopped", error=False)

        print("[RobotPickup] Simulation stopped")

    def _on_physics_step(self, dt):
        """Called every physics step while simulation is running."""
        if not self._is_running or not self._controller:
            return

        try:
            # Get current joint positions
            current_joint_positions = self._robot.get_joint_positions()

            # Compute actions from controller
            actions = self._controller.forward(
                picking_position=self._pickup_position,
                placing_position=self._place_position,
                current_joint_positions=current_joint_positions,
                end_effector_offset=self._end_effector_offset
            )

            # Apply actions
            if actions and self._articulation_controller:
                self._articulation_controller.apply_action(actions)

            # Check if task is complete
            if self._controller.is_done() and not self._task_complete:
                self._task_complete = True
                self._update_status("Task complete! Returning to home...", error=False)

                # Return to home after brief delay
                asyncio.ensure_future(self._return_to_home_async())

        except Exception as e:
            print(f"[RobotPickup] Physics step error: {e}")

    async def _return_to_home_async(self):
        """Return robot to home position after task completion."""
        await asyncio.sleep(2.0)  # Wait 2 seconds

        if self._home_joint_positions is not None:
            # Gradually move to home
            self._robot.set_joint_positions(self._home_joint_positions)

            await asyncio.sleep(3.0)  # Allow time to reach home

        # Stop simulation
        self._stop_simulation()
        self._update_status("✓ Complete! Robot returned to home. Click Start to repeat.", error=False)

    # ========================================================================
    # UI Helpers
    # ========================================================================

    def _update_status(self, message: str, error: bool = False):
        """Update status label."""
        if self._status_label:
            self._status_label.text = message
            color = 0xFFFF3333 if error else 0xFF00DD00
            self._status_label.style = {"font_size": 13, "color": color}

        print(f"[RobotPickup] {message}")
