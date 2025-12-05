"""
Test script to verify task ordering with same order values.

This test demonstrates that tasks with the same order value execute
in insertion order, not alphabetical order.
"""

import sys
import numpy as np

# Add module path
module_path = r"D:/poc/po_wiwynn_test/pochien.robotArm_sim/pochien/robotArm_sim/denso"
if module_path not in sys.path:
    sys.path.append(module_path)

from task_scheduler import TaskScheduler, ScheduledTask


def test_task_ordering():
    """Test that tasks with same order execute in insertion order"""

    print("="*70)
    print("TASK ORDERING TEST")
    print("="*70)

    # Create a minimal scheduler (without actual robot objects)
    scheduler = TaskScheduler()

    # Create mock tasks with names that would sort differently alphabetically
    # If sorted alphabetically: task10, task2, task_a, task_b, task_z
    # If sorted by insertion: task_z, task_b, task2, task10, task_a

    print("\nAdding tasks in this order:")
    print("  1. task_z (order=0)")
    print("  2. task_b (order=0)")
    print("  3. task2 (order=0)")
    print("  4. task10 (order=0)")
    print("  5. task_a (order=0)")

    # Add tasks in non-alphabetical order, all with order=0
    for task_id in ["task_z", "task_b", "task2", "task10", "task_a"]:
        task = ScheduledTask(
            task_id=task_id,
            setup=None,
            robot=None,
            controller=None,
            articulation_controller=None,
            robot_position=np.array([0.0, 0.0, 0.0]),
            enabled=True,
            order=0,
            pause_after=0.0,
            insertion_index=scheduler._insertion_counter
        )
        scheduler.tasks[task_id] = task
        scheduler._insertion_counter += 1

    # Rebuild task order
    scheduler._rebuild_task_order()

    print("\n" + "="*70)
    print("RESULTS")
    print("="*70)

    print("\nExpected execution order (insertion order):")
    print("  task_z → task_b → task2 → task10 → task_a")

    print("\nActual execution order:")
    print(f"  {' → '.join(scheduler.task_order)}")

    # Verify
    expected = ["task_z", "task_b", "task2", "task10", "task_a"]
    if scheduler.task_order == expected:
        print("\n✓ TEST PASSED: Tasks execute in insertion order!")
    else:
        print("\n✗ TEST FAILED: Tasks do NOT execute in insertion order")
        print(f"  Expected: {expected}")
        print(f"  Got:      {scheduler.task_order}")

    print("\n" + "="*70)
    print("ADDITIONAL TEST: Mixed order values")
    print("="*70)

    # Test with mixed order values
    scheduler2 = TaskScheduler()

    print("\nAdding tasks with different order values:")
    tasks_config = [
        ("task_c", 1),
        ("task_a", 0),
        ("task_b", 0),
        ("task_d", 1),
    ]

    for task_id, order in tasks_config:
        print(f"  - {task_id} (order={order})")
        task = ScheduledTask(
            task_id=task_id,
            setup=None,
            robot=None,
            controller=None,
            articulation_controller=None,
            robot_position=np.array([0.0, 0.0, 0.0]),
            enabled=True,
            order=order,
            pause_after=0.0,
            insertion_index=scheduler2._insertion_counter
        )
        scheduler2.tasks[task_id] = task
        scheduler2._insertion_counter += 1

    scheduler2._rebuild_task_order()

    print("\nExpected execution order:")
    print("  task_a (order=0) → task_b (order=0) → task_c (order=1) → task_d (order=1)")

    print("\nActual execution order:")
    actual_with_order = [f"{tid} (order={scheduler2.tasks[tid].order})" for tid in scheduler2.task_order]
    print(f"  {' → '.join(actual_with_order)}")

    expected2 = ["task_a", "task_b", "task_c", "task_d"]
    if scheduler2.task_order == expected2:
        print("\n✓ TEST PASSED: Tasks execute correctly with mixed order values!")
    else:
        print("\n✗ TEST FAILED")
        print(f"  Expected: {expected2}")
        print(f"  Got:      {scheduler2.task_order}")

    print("="*70)


if __name__ == "__main__":
    test_task_ordering()
