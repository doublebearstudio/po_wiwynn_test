"""
Robot Arm Simulation Extension

Simple extension for pick-and-place tasks using the simplified setup approach.
Uses PickPlaceSceneSetup class for easy scene configuration.
"""

import omni.ext
import omni.ui as ui
import numpy as np
import sys
import os

# Add denso directory to path for imports
current_dir = os.path.dirname(os.path.abspath(__file__))
denso_dir = os.path.join(current_dir, "denso")
if denso_dir not in sys.path:
    sys.path.insert(0, denso_dir)

# Import required modules
from simple_pick_place_setup import PickPlaceSceneSetup
from pick_place_controller import PickPlaceController
from isaacsim.core.utils.stage import add_reference_to_stage
from omni.timeline import get_timeline_interface
import omni.physx as _physx


class RobotArmSimulationExtension(omni.ext.IExt):
    """Robot Arm Simulation Extension with simplified setup."""

    def on_startup(self, ext_id: str):
        """Initialize the extension and create UI."""
        print("[RobotArmSim] Extension startup")

        # State variables
        self._setup = None
        self._robot = None
        self._controller = None
        self._articulation_controller = None
        self._physics_subscription = None
        self._timeline = get_timeline_interface()
        self._scene_setup_complete = False
        self._initialized = False
        self._was_playing = False

        # Configuration
        self._initial_position = np.array([-0.5, 0.4, 0.125])
        self._target_position = np.array([-0.6, -0.5, 0.135])
        self._custom_usd_path = "D:/poc/po_wiwynn_test/tst_cylinder01.usda"
        self._table_path = "D:/poc/po_wiwynn_test/prp_table01.usda"

        # Build UI
        self._build_ui()

    def _build_ui(self):
        """Build the UI window."""
        self._window = ui.Window("Robot Arm Simulation", width=400, height=300)

        with self._window.frame:
            with ui.VStack(spacing=15, height=0):
                # Title
                ui.Label(
                    "Robot Arm Simulation",
                    style={"font_size": 24, "color": 0xFFFFFFFF},
                    alignment=ui.Alignment.CENTER,
                    height=40
                )

                ui.Separator()

                # Status
                with ui.VStack(spacing=5):
                    ui.Label("Status:", style={"font_size": 14})
                    self._status_label = ui.Label(
                        "Ready - Click 'Set Up Scene' to begin",
                        word_wrap=True,
                        style={"font_size": 12, "color": 0xFF00DD00}
                    )

                ui.Spacer(height=10)

                # Set Up Scene Button
                self._setup_button = ui.Button(
                    "Set Up Scene",
                    clicked_fn=self._on_setup_scene_clicked,
                    height=60,
                    style={
                        "Button": {
                            "background_color": 0xFF2277FF,
                            "border_radius": 5,
                            "font_size": 18
                        }
                    }
                )

                ui.Spacer(height=10)

                # Start Task Button
                self._start_button = ui.Button(
                    "Start Task",
                    clicked_fn=self._on_start_task_clicked,
                    height=60,
                    enabled=False,
                    style={
                        "Button": {
                            "background_color": 0xFF228822,
                            "border_radius": 5,
                            "font_size": 18
                        },
                        "Button:disabled": {
                            "background_color": 0xFF555555
                        }
                    }
                )

                ui.Spacer(height=10)

                # Reset Scene Button
                self._reset_button = ui.Button(
                    "Reset Scene",
                    clicked_fn=self._on_reset_scene_clicked,
                    height=40,
                    style={
                        "Button": {
                            "background_color": 0xFFDD6600,
                            "border_radius": 5,
                            "font_size": 14
                        }
                    }
                )

    def on_shutdown(self):
        """Clean up extension resources."""
        print("[RobotArmSim] Extension shutdown")

        # Unsubscribe from physics updates
        if self._physics_subscription:
            self._physics_subscription.unsubscribe()
            self._physics_subscription = None

        # Close window
        if self._window:
            self._window.destroy()
            self._window = None

    # ========================================================================
    # Button Callbacks
    # ========================================================================

    def _on_setup_scene_clicked(self):
        """Handle Set Up Scene button - Creates the scene."""
        print("[RobotArmSim] Setting up scene...")
        self._update_status("Setting up scene...")

        try:
            # Create setup helper
            self._setup = PickPlaceSceneSetup(
                custom_usd_path=self._custom_usd_path,
                object_initial_position=self._initial_position,
                target_position=self._target_position
            )

            # Add table
            print("[RobotArmSim] Adding table...")
            add_reference_to_stage(usd_path=self._table_path, prim_path="/World/Table")

            # Create robot
            print("[RobotArmSim] Creating robot...")
            self._robot = self._setup.create_robot()

            # Create pickup object
            print("[RobotArmSim] Creating pickup object...")
            self._setup.create_object(mass=0.01)

            # Create target marker
            print("[RobotArmSim] Creating target marker...")
            self._setup.create_target_marker()

            # Initialize controller
            print("[RobotArmSim] Initializing controller...")
            self._controller = PickPlaceController(
                name="pick_place_controller",
                robot_articulation=self._robot,
                gripper=self._robot.gripper
            )

            self._articulation_controller = self._robot.get_articulation_controller()

            # Unsubscribe from any existing physics subscription (safety check)
            if self._physics_subscription:
                print("[RobotArmSim] Warning: Cleaning up old physics subscription")
                self._physics_subscription.unsubscribe()
                self._physics_subscription = None

            # Subscribe to physics updates
            self._physics_subscription = _physx.get_physx_interface().subscribe_physics_step_events(
                self._on_physics_step
            )

            self._scene_setup_complete = True
            self._update_status("Scene setup complete! Click 'Start Task' to begin.")

            # Enable Start Task button
            self._start_button.enabled = True
            self._setup_button.enabled = False

            print("[RobotArmSim] Scene setup complete")

        except Exception as e:
            self._update_status(f"Error: {str(e)}", error=True)
            print(f"[RobotArmSim] Setup error: {e}")
            import traceback
            traceback.print_exc()

            # Clean up on error
            self._reset_scene()

    def _on_start_task_clicked(self):
        """Handle Start Task button - Starts/stops the simulation."""
        if not self._scene_setup_complete:
            self._update_status("Error: Scene not set up!", error=True)
            return

        # Check current timeline state
        is_playing = self._timeline.is_playing()

        if is_playing:
            # Currently playing - stop it
            print("[RobotArmSim] Stopping simulation...")
            self._timeline.stop()

            # Clean up task resources
            self._cleanup_task()

            self._start_button.text = "Start Task"
            self._update_status("Simulation stopped - Task cleaned up")
        else:
            # Currently stopped - start it
            print("[RobotArmSim] Starting simulation...")
            self._timeline.play()
            self._start_button.text = "Stop Task"
            self._update_status("Simulation running...")

    def _on_reset_scene_clicked(self):
        """Handle Reset Scene button - Resets the entire scene."""
        print("[RobotArmSim] Reset Scene button clicked")

        # Stop timeline if playing
        if self._timeline.is_playing():
            self._timeline.stop()

        # Reset everything
        self._reset_scene()

        self._update_status("Scene reset - Click 'Set Up Scene' to begin")

    # ========================================================================
    # Physics Step Callback
    # ========================================================================

    def _on_physics_step(self, step_size):
        """Called every physics step."""
        if not self._scene_setup_complete:
            return

        # Detect timeline stop/start cycle
        is_playing = self._timeline.is_playing()

        if not is_playing:
            # Timeline stopped - reset initialization flag
            if self._was_playing:
                self._initialized = False
                self._was_playing = False
            return

        # Timeline is playing
        self._was_playing = True

        # Initialize/re-initialize when timeline starts
        if not self._initialized:
            try:
                # Initialize robot (creates the physics view)
                self._robot.initialize()

                # Wait for physics view to be created
                joint_positions = self._robot.get_joint_positions()
                if joint_positions is None:
                    # Physics view not ready yet, try again next frame
                    return

                # Physics view is ready, now initialize controller
                self._controller.reset()
                self._initialized = True
                print("[RobotArmSim] System initialized and running")
                self._update_status("Running pick and place task...")
            except AttributeError:
                # Physics context not ready yet - this is normal at startup
                # Silently wait and try again next frame
                return
            except Exception as e:
                # Other unexpected errors - log them
                print(f"[RobotArmSim] Initialization error: {e}")
                import traceback
                traceback.print_exc()
                return

        # Main control loop
        try:
            # Get joint positions
            joint_positions = self._robot.get_joint_positions()

            # Safety check
            if joint_positions is None:
                print("[RobotArmSim] Warning: Physics view lost, re-initializing...")
                self._initialized = False
                return

            # Get observations
            observations = self._setup.get_observations(self._robot)

            # Compute actions
            actions = self._controller.forward(
                picking_position=observations["pickup_object"]["position"],
                placing_position=observations["pickup_object"]["target_position"],
                current_joint_positions=joint_positions,
                end_effector_offset=np.array([0, 0, 0.25]),
            )

            # Check completion
            if self._controller.is_done():
                print("[RobotArmSim] Pick and place complete!")
                self._update_status("Task complete!")

            # Apply actions
            self._articulation_controller.apply_action(actions)

        except Exception as e:
            print(f"[RobotArmSim] Error in control loop: {e}")
            import traceback
            traceback.print_exc()

            # Stop timeline and clean up on critical error
            self._update_status(f"Error: {str(e)}", error=True)
            self._timeline.stop()
            if self._physics_subscription:
                self._physics_subscription.unsubscribe()
                self._physics_subscription = None
            self._initialized = False
            self._start_button.text = "Start Task"

    # ========================================================================
    # Cleanup Methods
    # ========================================================================

    def _cleanup_task(self):
        """Clean up task resources and reset state."""
        print("[RobotArmSim] Cleaning up task...")

        # Unsubscribe from physics updates
        if self._physics_subscription:
            self._physics_subscription.unsubscribe()
            self._physics_subscription = None
            print("[RobotArmSim] Physics subscription removed")

        # Reset state flags
        self._initialized = False
        self._was_playing = False

        print("[RobotArmSim] Task cleaned up")

    def _reset_scene(self):
        """Reset scene setup to allow re-initialization."""
        print("[RobotArmSim] Resetting scene...")

        # Clean up task first
        self._cleanup_task()

        # Reset scene objects
        self._setup = None
        self._robot = None
        self._controller = None
        self._articulation_controller = None
        self._scene_setup_complete = False

        # Re-enable setup button
        self._setup_button.enabled = True
        self._start_button.enabled = False

        print("[RobotArmSim] Scene reset complete")

    # ========================================================================
    # Helper Methods
    # ========================================================================

    def _update_status(self, message: str, error: bool = False):
        """Update status label."""
        if self._status_label:
            self._status_label.text = message
            color = 0xFFFF3333 if error else 0xFF00DD00
            self._status_label.style = {"font_size": 12, "color": color}

        print(f"[RobotArmSim] {message}")
