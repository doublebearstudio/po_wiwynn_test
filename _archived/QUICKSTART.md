# Quick Start Guide - Robot Pick and Place

Get started with the robot pick and place scripts in under 5 minutes!

## Files Created

```
D:\poc\po_wiwynn_test\
├── po_wiwynn_test_v0002.usda          # Your USD scene file
├── robot_pick_place.py                 # Basic version
├── robot_pick_place_advanced.py        # Advanced version
├── robot_pick_place_production.py      # ⭐ RECOMMENDED - Production version
├── config.py                           # Configuration file
├── README_ROBOT_CONTROL.md             # Detailed documentation
└── QUICKSTART.md                       # This file
```

## Which Script Should I Use?

### 🎯 **RECOMMENDED: robot_pick_place_production.py**
- Uses `config.py` for easy customization
- Production-ready with detailed logging
- Best for actual use

### 📚 Other Options:
- `robot_pick_place.py` - Basic implementation for learning
- `robot_pick_place_advanced.py` - Advanced features, hardcoded settings

## Quick Start (3 Steps)

### Step 1: Adjust Configuration (Optional)

Edit `config.py` to customize behavior:

```python
# Quick settings to adjust:
DANNY_PLACEMENT_OFFSET["z"] = 0.40    # How high to place lid on Danny
BOX_BODY_PLACEMENT_OFFSET["z"] = 0.20 # How high to place cube in box
MOTION_STEPS["approach"] = 150         # Motion smoothness (higher = smoother)
DEBUG_MODE = True                      # Enable/disable debug output
```

### Step 2: Run the Script

**Windows:**
```cmd
cd C:\Users\YourName\AppData\Local\ov\pkg\isaac_sim-2023.1.0
python.bat D:\poc\po_wiwynn_test\robot_pick_place_production.py
```

**Linux:**
```bash
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.0
./python.sh ~/poc/po_wiwynn_test/robot_pick_place_production.py
```

### Step 3: Watch It Work!

The script will automatically:
1. ✅ Robot 2 picks box lid → places on Danny table
2. ✅ Robot 1 picks cube → places in box body
3. ✅ Both robots return to starting positions

## Expected Output

```
======================================================================
               STARTING PRODUCTION SEQUENCE
======================================================================

======================================================================
TASK 1: Robot 2 - Pick Box Lid and Place on Danny Table
======================================================================

────────────────────────────────────────────────────────────
  Picking box_lid with Robot 2
────────────────────────────────────────────────────────────
  → Approaching above object...
  Opening Robot 2 gripper
  → Moving to grasp position...
  Closing Robot 2 gripper
  → Lifting object...
  ✓ box_lid picked successfully

────────────────────────────────────────────────────────────
  Placing object with Robot 2
────────────────────────────────────────────────────────────
  → Approaching target location...
  → Moving to place position...
  Opening Robot 2 gripper
  → Retracting...
  ✓ Object placed successfully

[✓] TASK 1 COMPLETED

... (similar output for Task 2 and 3) ...

======================================================================
                    SEQUENCE COMPLETE!
======================================================================
```

## Customization Examples

### Make motions slower/smoother
```python
# In config.py
MOTION_STEPS = {
    "approach": 250,  # Increase from 150
    "grasp": 150,     # Increase from 100
    # ... etc
}
```

### Adjust placement positions
```python
# In config.py
DANNY_PLACEMENT_OFFSET = {
    "x": 0.0,
    "y": 0.1,      # Move placement 10cm in Y direction
    "z": 0.50,     # Place 50cm above Danny instead of 40cm
}
```

### Run without GUI (headless mode)
```python
# In config.py
SIMULATION_CONFIG = {
    "headless": True,   # Change from False
    "width": 1920,
    "height": 1080,
}
```

### Enable only specific tasks
```python
# In config.py
ENABLE_TASKS = {
    "task1_robot2_boxlid": True,   # Enable
    "task2_robot1_cube": False,    # Disable this task
    "task3_return_home": True,     # Enable
}
```

## Troubleshooting

### ❌ Error: "USD file not found"
**Solution:** Update path in `config.py`
```python
USD_STAGE_PATH = "D:/poc/po_wiwynn_test/po_wiwynn_test_v0002.usda"
```

### ❌ Error: "config.py not found"
**Solution:** Ensure `config.py` is in the same directory as the Python script

### ❌ Robots not moving
**Check:**
1. Is physics enabled in the scene?
2. Are joint limits correct?
3. Is simulation stepping? (check console output)

### ❌ Objects falling or not grasping
**Note:** Current implementation uses placeholder gripper control
- Actual gripper control needs to be implemented based on your gripper type
- See `README_ROBOT_CONTROL.md` for implementation details

## What's Next?

### For Better Results:
1. **Implement proper IK** - Integrate Lula IK solver (see README)
2. **Add gripper control** - Implement actual gripper joint control
3. **Enable collision detection** - Add safety checks
4. **Use motion planning** - Integrate RMPflow for smoother paths

### Learn More:
- Read `README_ROBOT_CONTROL.md` for detailed documentation
- Check Isaac Sim docs: https://docs.omniverse.nvidia.com/isaacsim/latest/
- Explore examples: `isaac_sim/standalone_examples/api/omni.isaac.manipulators/`

## Task Sequence Summary

```
┌─────────────────────────────────────────────────────────────┐
│  TASK 1: Robot 2 (kr210_l150_02)                           │
│  ┌─────────────┐                    ┌──────────────┐       │
│  │ geo_boxLid  │  ──────────────>   │ Danny Table  │       │
│  │    _01      │      Robot 2       │              │       │
│  └─────────────┘                    └──────────────┘       │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  TASK 2: Robot 1 (kr210_l150_01)                           │
│  ┌─────────────┐                    ┌──────────────┐       │
│  │ geo_cube_01 │  ──────────────>   │ grp_boxBody  │       │
│  │             │      Robot 1       │     _01      │       │
│  └─────────────┘                    └──────────────┘       │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  TASK 3: Return to Home                                     │
│  Both robots return to their starting positions             │
└─────────────────────────────────────────────────────────────┘
```

## Support

- 📖 Full docs: `README_ROBOT_CONTROL.md`
- 🔧 Configuration: `config.py`
- 💬 Isaac Sim forums: https://forums.developer.nvidia.com/c/omniverse/simulation/69

---

**Ready to go!** Run `robot_pick_place_production.py` to start the sequence.
