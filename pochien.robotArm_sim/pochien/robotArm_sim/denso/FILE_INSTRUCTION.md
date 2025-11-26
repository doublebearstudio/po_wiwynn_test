# FILE_INSTRUCTION.md

# Issac Sim version: 4.5.0

## Files need for pick place task

# Operation files
pick_up_script_editor.py: execution code for pick place task

simple_pick_place_setup.py: information query and scene setup

pick_place_controller.py: Custom PickPlaceController that uses RMPflow motion planning for the Denso Cobotta robot's pick-and-place operations in Isaac Sim.

rmpflow.py: rmpflow definition file

# Asset files
robot asset: robot_asset = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Robots/Denso/cobotta_pro_900.usd"

robot description file: "D:/poc/po_wiwynn_test/pochien.robotArm_sim/pochien/robotArm_sim/denso/robot_descriptor.yaml"

rmpflow_config file: "D:/poc/po_wiwynn_test/pochien.robotArm_sim/pochien/robotArm_sim/denso/denso_rmpflow_common.yaml"

urdf file: "D:/isaac_sim_v4_5_0/extscache/isaacsim.asset.importer.urdf-2.3.10+106.4.0.wx64.r.cp310/data/urdf/robots/cobotta_pro_900/cobotta_pro_900.urdf"

target object (needs coolider preset being set ): "D:/poc/po_wiwynn_test/tst_cylinder01.usda"

misc scene object (i.e. table, light): "D:/poc/po_wiwynn_test/prp_table01.usda"

