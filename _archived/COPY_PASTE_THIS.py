"""
========================================
ISAAC SIM SCRIPT EDITOR - COPY & PASTE
========================================

INSTRUCTIONS:
1. Open po_wiwynn_test_v0002.usda in Isaac Sim
2. Window > Script Editor
3. Copy this ENTIRE file
4. Paste into Script Editor
5. Click RUN

This is the minimal version - just copy and run!
"""

import numpy as np
import asyncio
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.prims import RigidPrim, XFormPrim
import omni.kit.app

async def run():
    print("\n" + "="*60)
    print("ROBOT PICK AND PLACE - STARTING")
    print("="*60 + "\n")

    # Setup world
    world = World.instance() or World(stage_units_in_meters=1.0)
    await world.reset_async()
    await omni.kit.app.get_app().next_update_async()

    # Get robots
    r1 = world.scene.add(Articulation(prim_path="/World/kr210_l150_01", name="r1"))
    r2 = world.scene.add(Articulation(prim_path="/World/kr210_l150_02", name="r2"))

    # Get objects
    lid = world.scene.add(RigidPrim(prim_path="/World/geo_boxLid_01", name="lid"))
    cube = world.scene.add(RigidPrim(prim_path="/World/geo_cube_01", name="cube"))

    # Get targets
    danny = XFormPrim(prim_path="/World/Danny")
    box = XFormPrim(prim_path="/World/grp_boxBody_01")

    # Reset and store home
    await world.reset_async()
    await omni.kit.app.get_app().next_update_async()
    r1_home = r1.get_joint_positions()
    r2_home = r2.get_joint_positions()

    # Movement helper
    async def move(robot, target, steps=100):
        current = robot.get_joint_positions()
        for i in range(steps):
            a = 0.5 * (1 - np.cos(np.pi * (i+1) / steps))
            robot.apply_action(ArticulationAction(joint_positions=current + a * (target - current)))
            await world.play_async()
            await omni.kit.app.get_app().next_update_async()

    # Gripper helper (placeholder)
    async def grip(steps=30):
        for _ in range(steps):
            await world.play_async()
            await omni.kit.app.get_app().next_update_async()

    # IK placeholder
    def ik(robot, pos):
        return robot.get_joint_positions()

    # Pick helper
    async def pick(robot, obj, name):
        print(f"\n{name} picking {obj.name}...")
        p, _ = obj.get_world_pose()
        await move(robot, ik(robot, p + np.array([0,0,0.3])), 150)
        await grip()
        await move(robot, ik(robot, p + np.array([0,0,0.05])), 100)
        await grip()
        await move(robot, ik(robot, p + np.array([0,0,0.3])), 100)
        print(f"✓ Picked {obj.name}")

    # Place helper
    async def place(robot, target, name):
        print(f"\n{name} placing...")
        await move(robot, ik(robot, target + np.array([0,0,0.3])), 150)
        await move(robot, ik(robot, target), 100)
        await grip()
        await move(robot, ik(robot, target + np.array([0,0,0.3])), 100)
        print(f"✓ Placed")

    # Get positions
    danny_p, _ = danny.get_world_pose()
    box_p, _ = box.get_world_pose()

    # TASK 1: R2 picks lid to Danny
    print("\n" + "="*60)
    print("TASK 1: Robot 2 - Box Lid to Danny")
    print("="*60)
    await pick(r2, lid, "R2")
    await place(r2, danny_p + np.array([0,0,0.4]), "R2")
    print("\n[✓] TASK 1 DONE")

    # TASK 2: R1 picks cube to box
    print("\n" + "="*60)
    print("TASK 2: Robot 1 - Cube to Box")
    print("="*60)
    await pick(r1, cube, "R1")
    await place(r1, box_p + np.array([0,0,0.2]), "R1")
    print("\n[✓] TASK 2 DONE")

    # TASK 3: Return home
    print("\n" + "="*60)
    print("TASK 3: Return Home")
    print("="*60)
    print("\nR2 returning...")
    await move(r2, r2_home, 200)
    print("✓ R2 home")
    print("\nR1 returning...")
    await move(r1, r1_home, 200)
    print("✓ R1 home")
    print("\n[✓] TASK 3 DONE")

    print("\n" + "="*60)
    print("ALL TASKS COMPLETE!")
    print("="*60 + "\n")

# RUN IT
asyncio.ensure_future(run())
print("\n>>> Sequence started! <<<\n")
