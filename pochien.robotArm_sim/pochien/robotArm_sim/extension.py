# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION or
# its affiliates is strictly prohibited.

"""
Robot Pick-and-Place Extension - Simple Setup

This extension provides a single button to set up the pick-and-place scene.
After clicking the button, start the simulation by pressing the PLAY button in Isaac Sim.

Features:
- Setup Scene button: Configures robot, object, and controller
- Uses PickPlaceSceneSetup for scene creation
- Automatic control loop via physics subscription
- Start/Stop simulation with Isaac Sim's PLAY/STOP buttons
- Properly resets when stopping and restarting

Author: Generated for Wiwynn Test
Date: 2025-11-28
"""

import omni.ext
import omni.ui as ui
import omni.usd
from isaacsim.core.utils.stage import add_reference_to_stage
from omni.timeline import get_timeline_interface
import omni.physx as _physx
import numpy as np
import sys
import os
import carb

# Add denso directory to path for imports
current_dir = os.path.dirname(os.path.abspath(__file__))
denso_dir = os.path.join(current_dir, "denso")
if denso_dir not in sys.path:
    sys.path.insert(0, denso_dir)

# Import from local denso directory
try:
    from simple_pick_place_setup import PickPlaceSceneSetup
    from pick_place_controller import PickPlaceController
except ImportError as e:
    print(f"Warning: Could not import required modules: {e}")
    PickPlaceSceneSetup = None
    PickPlaceController = None


class MyExtension(omni.ext.IExt):
    """
    Extension for robot pick-and-place scene setup.

    Provides a simple interface to set up the scene, then use Isaac Sim's
    PLAY button to start the simulation.
    """

    def on_startup(self, ext_id: str):
        """Initialize the extension and create UI."""
        print("[RobotPickup] Extension startup")

        # State variables
        self._setup = None
        self._robot = None
        self._controller = None
        self._articulation_controller = None
        self._physics_subscription = None
        self._timeline_subscription = None
        self._initialized = False
        self._scene_setup_complete = False
        self._is_playing = False

        # Scene configuration
        self._initial_position = np.array([-0.5, 0.4, 0.125])
        self._target_position = np.array([-0.6, -0.5, 0.135])
        self._custom_usd_path = "D:/poc/po_wiwynn_test/tst_cylinder01.usda"
        self._table_path = "D:/poc/po_wiwynn_test/prp_table01.usda"

        # Get timeline interface
        self._timeline = get_timeline_interface()

        # Create UI
        self._build_ui()

    def _build_ui(self):
        """Build the UI window with Setup Scene button."""
        self._window = ui.Window("Robot Arm Simulator", width=400, height=450)

        with self._window.frame:
            with ui.VStack(spacing=15, height=0):
                # Header
                ui.Label(
                    "Pick & Place Scene Setup",
                    style={"font_size": 22, "color": 0xFFFFFFFF},
                    alignment=ui.Alignment.CENTER,
                    height=30
                )

                ui.Separator()

                ui.Spacer(height=5)

                # New Scene and Setup Scene buttons (horizontal)
                with ui.HStack(spacing=10):
                    ui.Button(
                        "New Scene",
                        clicked_fn=self._on_empty_scene_clicked,
                        height=60,
                        style={
                            "Button": {
                                "background_color": 0xFF666666,
                                "border_radius": 5,
                                "font_size": 16
                            }
                        }
                    )

                    self._setup_button = ui.Button(
                        "Setup Scene",
                        clicked_fn=self._on_setup_clicked,
                        height=60,
                        style={
                            "Button": {
                                "background_color": 0xFF228822,
                                "border_radius": 5,
                                "font_size": 18
                            }
                        }
                    )

                ui.Spacer(height=5)

                # Start/Stop Button
                self._start_stop_button = ui.Button(
                    "Start Simulation",
                    clicked_fn=self._on_start_stop_clicked,
                    height=50,
                    enabled=False,
                    style={
                        "Button": {
                            "background_color": 0xFF2288FF,
                            "border_radius": 5,
                            "font_size": 16
                        }
                    }
                )

                ui.Spacer(height=10)

                # Scene Parameters
                with ui.CollapsableFrame("Scene Parameters", height=0, collapsed=True):
                    with ui.VStack(spacing=3):
                        ui.Label("Pickup Position (m):", style={"font_size": 11})
                        ui.Label(
                            f"  X: {self._initial_position[0]:.3f}, "
                            f"Y: {self._initial_position[1]:.3f}, "
                            f"Z: {self._initial_position[2]:.3f}",
                            style={"font_size": 10, "color": 0xFFAAAAAA}
                        )

                        ui.Spacer(height=5)

                        ui.Label("Target Position (m):", style={"font_size": 11})
                        ui.Label(
                            f"  X: {self._target_position[0]:.3f}, "
                            f"Y: {self._target_position[1]:.3f}, "
                            f"Z: {self._target_position[2]:.3f}",
                            style={"font_size": 10, "color": 0xFFAAAAAA}
                        )

                # Robot Parameters
                with ui.CollapsableFrame("Robot Parameters", height=0, collapsed=True):
                    with ui.VStack(spacing=3):
                        ui.Label("Robot Prim Path:", style={"font_size": 11})
                        ui.Label(
                            "  /World/cobotta_pro_900",
                            style={"font_size": 10, "color": 0xFFAAAAAA}
                        )

                        ui.Spacer(height=5)

                        ui.Label("End Effector:", style={"font_size": 11})
                        ui.Label(
                            "  onrobot_rg6_base_link",
                            style={"font_size": 10, "color": 0xFFAAAAAA}
                        )

                        ui.Spacer(height=5)

                        ui.Label("Gripper Type:", style={"font_size": 11})
                        ui.Label(
                            "  Parallel Gripper (OnRobot RG6)",
                            style={"font_size": 10, "color": 0xFFAAAAAA}
                        )

                ui.Spacer(height=10)

                # Status Section (at the bottom)
                ui.Label("Status:", style={"font_size": 12, "color": 0xFFAAAAFF})
                with ui.VStack(spacing=5):
                    self._status_label = ui.Label(
                        "Ready - Click 'Setup Scene' to begin",
                        word_wrap=True,
                        style={"font_size": 13, "color": 0xFF00DD00},
                        height=60
                    )

    def on_shutdown(self):
        """Clean up extension resources."""
        print("[RobotPickup] Extension shutdown")

        # Unsubscribe from timeline events
        if self._timeline_subscription:
            self._timeline_subscription.unsubscribe()
            self._timeline_subscription = None

        # Unsubscribe from physics
        if self._physics_subscription:
            self._physics_subscription.unsubscribe()
            self._physics_subscription = None

        # Close window
        if self._window:
            self._window.destroy()
            self._window = None

    def _on_empty_scene_clicked(self):
        """Handle New Scene button - Create a new empty stage."""
        print("[RobotPickup] New Scene button clicked")

        try:
            # Stop simulation if running
            if self._is_playing:
                self._timeline.stop()

            # Clear subscriptions
            if self._timeline_subscription:
                self._timeline_subscription.unsubscribe()
                self._timeline_subscription = None

            if self._physics_subscription:
                self._physics_subscription.unsubscribe()
                self._physics_subscription = None

            # Reset state
            self._setup = None
            self._robot = None
            self._controller = None
            self._articulation_controller = None
            self._initialized = False
            self._scene_setup_complete = False
            self._is_playing = False

            # Create new empty stage
            omni.usd.get_context().new_stage()

            # Re-enable setup button and restore green color
            self._setup_button.enabled = True
            self._setup_button.style = {
                "Button": {
                    "background_color": 0xFF228822,
                    "border_radius": 5,
                    "font_size": 18
                }
            }
            self._start_stop_button.enabled = False
            self._start_stop_button.text = "Start Simulation"
            self._start_stop_button.style = {
                "Button": {
                    "background_color": 0xFF2288FF,
                    "border_radius": 5,
                    "font_size": 16
                }
            }

            self._update_status("New Scene created. Click 'Setup Scene' to begin.", error=False)
            print("[RobotPickup] New Scene created successfully")

        except Exception as e:
            self._update_status(f"Error creating New Scene: {str(e)}", error=True)
            print(f"[RobotPickup] New Scene error: {e}")
            import traceback
            traceback.print_exc()

    def _on_setup_clicked(self):
        """Handle Setup Scene button - Set up the entire pick-and-place scene."""
        print("[RobotPickup] Setup Scene button clicked")

        if self._scene_setup_complete:
            self._update_status("Scene already set up. Press PLAY to start.", error=False)
            return

        try:
            self._update_status("Setting up scene...", error=False)

            # Check if required classes are available
            if not PickPlaceSceneSetup or not PickPlaceController:
                self._update_status("Error: Required modules not available!", error=True)
                return

            print("="*70)
            print("PICK AND PLACE SETUP")
            print("="*70)

            # Create setup helper
            print("\n→ Setting up scene...")
            self._setup = PickPlaceSceneSetup(
                custom_usd_path=self._custom_usd_path,
                object_initial_position=self._initial_position,
                target_position=self._target_position
            )

            # Add table
            self._update_status("Adding table...", error=False)
            add_reference_to_stage(usd_path=self._table_path, prim_path="/World/Table")

            # Create robot
            self._update_status("Creating robot...", error=False)
            print("  → Creating robot...")
            self._robot = self._setup.create_robot()

            # Create pickup object
            self._update_status("Creating pickup object...", error=False)
            print("  → Creating pickup object...")
            self._setup.create_object(mass=0.01)  # 10 grams

            # Create target marker
            self._update_status("Creating target marker...", error=False)
            print("  → Creating target marker...")
            self._setup.create_target_marker()

            print("✓ Scene setup complete")

            # Initialize controller
            self._update_status("Initializing controller...", error=False)
            print("\n→ Initializing controller...")

            self._controller = PickPlaceController(
                name="pick_place_controller",
                robot_articulation=self._robot,
                gripper=self._robot.gripper,
                events_dt=[
                        0.005,  # 0: Move to pre-grasp position
                        0.05,  # 1: Descend to grasp height
                        1.0,    # 2: Wait/stabilize before grasping (important!)
                        0.05,   # 3: Close gripper
                        0.001, # 4: Wait after grasp
                        2.0,  # 5: Lift object
                        0.0005, # 6: Move to target position
                        0.01,    # 7: Lower to placement height
                        0.0008, # 8: Open gripper
                        0.008   # 9: Retreat from placed object
                        ]
            )

            self._articulation_controller = self._robot.get_articulation_controller()

            print("✓ Controller ready")

            # Subscribe to physics updates
            self._update_status("Setting up physics subscription...", error=False)
            self._physics_subscription = _physx.get_physx_interface().subscribe_physics_step_events(
                self._simulation_step
            )

            # Subscribe to timeline events to detect play/stop
            self._timeline_subscription = self._timeline.get_timeline_event_stream().create_subscription_to_pop(
                self._on_timeline_event
            )

            self._scene_setup_complete = True
            self._setup_button.enabled = False
            # Grey out the Setup Scene button
            self._setup_button.style = {
                "Button": {
                    "background_color": 0xFF555555,
                    "border_radius": 5,
                    "font_size": 18
                }
            }
            self._start_stop_button.enabled = True

            print("\n" + "="*70)
            print("READY")
            print("="*70)
            print("→ Click 'Start Simulation' button or press ▶ PLAY in Isaac Sim")
            print("="*70)

            self._update_status(
                "Setup complete! Click 'Start Simulation' or press PLAY in Isaac Sim.",
                error=False
            )

        except Exception as e:
            self._update_status(f"Error: {str(e)}", error=True)
            print(f"[RobotPickup] Setup error: {e}")
            import traceback
            traceback.print_exc()

    def _on_start_stop_clicked(self):
        """Handle Start/Stop button click."""
        if self._is_playing:
            # Stop simulation
            print("[RobotPickup] Stop Simulation button clicked")
            self._timeline.stop()
        else:
            # Start simulation
            print("[RobotPickup] Start Simulation button clicked")
            self._timeline.play()

    def _on_timeline_event(self, event):
        """Handle timeline events (play/stop/pause)."""
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            print("[RobotPickup] Timeline PLAY event detected")
            self._is_playing = True
            self._update_status("Simulation started. Initializing...", error=False)

            # Update button
            self._start_stop_button.text = "Stop Simulation"
            self._start_stop_button.style = {
                "Button": {
                    "background_color": 0xFFFF4444,
                    "border_radius": 5,
                    "font_size": 16
                }
            }

        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            print("[RobotPickup] Timeline STOP event detected")
            self._is_playing = False
            self._on_simulation_stop()

            # Update button
            self._start_stop_button.text = "Start Simulation"
            self._start_stop_button.style = {
                "Button": {
                    "background_color": 0xFF2288FF,
                    "border_radius": 5,
                    "font_size": 16
                }
            }

    def _on_simulation_stop(self):
        """Handle simulation stop - reset state for next run."""
        print("[RobotPickup] Resetting for next run...")

        # Reset initialization flag
        self._initialized = False

        # Reset task done flag
        if hasattr(self, '_task_done_logged'):
            delattr(self, '_task_done_logged')

        # Reset controller if it exists
        if self._controller:
            try:
                self._controller.reset()
                print("[RobotPickup] Controller reset")
            except Exception as e:
                print(f"[RobotPickup] Controller reset error: {e}")

        self._update_status(
            "Simulation stopped. Click 'Start Simulation' to run again.",
            error=False
        )

    def _simulation_step(self, step_size):
        """Called every physics step when simulation is playing."""

        # Only run if timeline is playing
        if not self._is_playing:
            return

        # One-time initialization when timeline starts
        if not self._initialized:
            try:
                self._robot.initialize()
                self._controller.reset()
                self._initialized = True
                print("\n✓ System initialized")
                print("="*70)
                print("RUNNING PICK AND PLACE")
                print("="*70)
                self._update_status("Running: Pick and place in progress...", error=False)
            except Exception as e:
                # Wait for physics context
                return

        # Main control loop
        try:
            # Get observations
            observations = self._setup.get_observations(self._robot)

            # Compute actions
            actions = self._controller.forward(
                picking_position=observations["pickup_object"]["position"],
                placing_position=observations["pickup_object"]["target_position"],
                current_joint_positions=observations["cobotta_robot"]["joint_positions"],
                end_effector_offset=np.array([0, 0, 0.25]),
            )

            # Check completion
            if self._controller.is_done():
                if not hasattr(self, '_task_done_logged'):
                    print("✓ Pick and place complete!")
                    self._update_status("Task complete!", error=False)
                    self._task_done_logged = True

            # Apply actions
            if actions:
                self._articulation_controller.apply_action(actions)

        except Exception as e:
            print(f"Error in control loop: {e}")

    def _update_status(self, message: str, error: bool = False):
        """Update status label."""
        if self._status_label:
            self._status_label.text = message
            color = 0xFFFF3333 if error else 0xFF00DD00
            self._status_label.style = {"font_size": 13, "color": color}

        print(f"[RobotPickup] {message}")
