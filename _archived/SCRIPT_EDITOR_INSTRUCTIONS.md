# Isaac Sim Script Editor - Quick Instructions

## Step-by-Step Guide

### Method 1: Simple Version (Recommended for Quick Testing)

1. **Open your USD file in Isaac Sim**
   - File > Open
   - Navigate to: `D:\poc\po_wiwynn_test\po_wiwynn_test_v0002.usda`
   - Click Open

2. **Open Script Editor**
   - Go to: **Window > Script Editor**
   - Or press: `Ctrl + Shift + E`

3. **Copy the script**
   - Open file: `script_editor_simple.py`
   - Select All (Ctrl+A) and Copy (Ctrl+C)

4. **Paste and Run**
   - Paste into Script Editor (Ctrl+V)
   - Click **"Run"** button at the top
   - Or press: `Ctrl + Enter`

5. **Watch the magic happen!**
   - The robots will automatically execute the pick and place sequence
   - Check the console output for progress updates

---

## Available Script Versions

### 📄 `script_editor_simple.py` ⭐ RECOMMENDED
- **Size:** ~200 lines
- **Best for:** Quick testing and learning
- **Features:** Simple, easy to understand code
- **Customization:** Edit values directly in the script

### 📄 `script_editor_version.py`
- **Size:** ~500 lines
- **Best for:** Production use
- **Features:** Full-featured with configuration
- **Customization:** Uses CONFIG dictionary at the top

---

## Quick Customization

### Change Motion Speed
Find this in the script:
```python
await move_robot(robot, target_joints, steps=100)
#                                      ↑
#                                   Change this number
#                                   Higher = slower/smoother
#                                   Lower = faster
```

### Change Placement Height
Find this in the script:
```python
danny_target = danny_pos + np.array([0, 0, 0.4])
#                                           ↑
#                                   Danny table height offset

box_target = box_pos + np.array([0, 0, 0.2])
#                                        ↑
#                                   Box body height offset
```

### Enable/Disable Debug Output
In `script_editor_version.py`:
```python
CONFIG = {
    "debug": True,  # Change to False to reduce console output
    ...
}
```

---

## Expected Console Output

```
============================================================
Starting Robot Pick and Place Sequence
============================================================

Getting robots...
Getting objects...
Home positions stored

============================================================
TASK 1: Robot 2 picks box lid to Danny table
============================================================

--- Robot 2 Picking box_lid ---
  Approaching...
  Opening gripper...
  Moving to grasp...
  Closing gripper...
  Lifting...
✓ box_lid picked

--- Robot 2 Placing Object ---
  Approaching...
  Moving to place...
  Releasing...
  Retracting...
✓ Placed

[✓] TASK 1 COMPLETE

============================================================
TASK 2: Robot 1 picks cube to box body
============================================================

--- Robot 1 Picking cube ---
  ...

[✓] TASK 2 COMPLETE

============================================================
TASK 3: Returning robots to home positions
============================================================

--- Robot 2 Returning Home ---
  Returning...
✓ Robot 2 at home

--- Robot 1 Returning Home ---
  Returning...
✓ Robot 1 at home

[✓] TASK 3 COMPLETE

============================================================
ALL TASKS COMPLETE!
============================================================
```

---

## Troubleshooting

### ❌ Error: "World is None"
**Solution:**
- Make sure the USD file is loaded
- Try running the script again

### ❌ Error: "Cannot find prim at path..."
**Solution:**
- Verify the USD file has all required objects:
  - `/World/kr210_l150_01` (Robot 1)
  - `/World/kr210_l150_02` (Robot 2)
  - `/World/geo_boxLid_01` (Box lid)
  - `/World/geo_cube_01` (Cube)
  - `/World/Danny` (Danny table)
  - `/World/grp_boxBody_01` (Box body)

### ❌ Robots not moving
**Check:**
1. Is the simulation playing? (Check the play button)
2. Are the robots already in motion? (Wait for completion)
3. Check console for errors

### ❌ Script runs but nothing happens
**Try:**
1. Click the **Play** button in Isaac Sim first
2. Then run the script
3. Check the viewport is showing the scene

---

## Tips for Best Results

### 🎯 Before Running:
1. **Frame the scene** - Press `F` to focus camera on objects
2. **Check physics** - Ensure PhysicsScene is active
3. **Save your work** - Script modifies robot positions

### 🎯 While Running:
1. **Don't interact** - Let the script complete
2. **Watch the console** - Monitor progress
3. **Observe viewport** - See robots in action

### 🎯 After Running:
1. **Reset if needed** - File > Reload to restore original state
2. **Adjust parameters** - Modify script and run again
3. **Save positions** - Note successful configurations

---

## Advanced: Running Custom Sequences

You can modify the script to create custom sequences:

```python
# Example: Only run Task 1
# Comment out Tasks 2 and 3:

# TASK 1 - Keep this
await pick(robot2, box_lid, "Robot 2")
await place(robot2, danny_target, "Robot 2")

# TASK 2 - Comment out
# await pick(robot1, cube, "Robot 1")
# await place(robot1, box_target, "Robot 1")

# TASK 3 - Comment out
# await move_robot(robot2, robot2_home, 200, "Returning")
# await move_robot(robot1, robot1_home, 200, "Returning")
```

---

## Comparison: Script Editor vs Standalone

| Feature | Script Editor | Standalone Python |
|---------|--------------|-------------------|
| GUI | Use Isaac Sim GUI | Launches own window |
| Setup | Scene already open | Must load scene in code |
| SimulationApp | Not needed | Required |
| Debugging | Interactive | Command line |
| Best for | Quick testing | Production/Automation |

---

## Next Steps

1. **Test with simple version** - Get familiar with the sequence
2. **Customize parameters** - Adjust speeds and positions
3. **Try full version** - Use `script_editor_version.py` for more control
4. **Add IK solver** - Integrate Lula IK for better motion
5. **Implement gripper** - Add actual gripper control

---

## Files Reference

```
D:\poc\po_wiwynn_test\
├── script_editor_simple.py          ⭐ Start here
├── script_editor_version.py         📦 Full featured
├── SCRIPT_EDITOR_INSTRUCTIONS.md    📖 This file
│
├── robot_pick_place_production.py   🚀 For standalone execution
├── config.py                        ⚙️ Configuration file
└── README_ROBOT_CONTROL.md          📚 Complete documentation
```

---

## Support & Resources

- **Isaac Sim Docs:** https://docs.omniverse.nvidia.com/isaacsim/latest/
- **Script Editor Guide:** Isaac Sim docs > Script Editor
- **Python API:** Isaac Sim docs > Core API
- **Forums:** https://forums.developer.nvidia.com/c/omniverse/simulation/69

---

**Ready to go!** Open Isaac Sim, load your scene, and run the script! 🚀
