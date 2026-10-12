# Local changes to franka_description (franka_ros 0.10.2, ~/catkin_ws/src/franka_ros)

This robot differs from a stock Panda + Franka Hand. MoveIt / TF / the API use
the URDF from franka_description, so the differences are patched there
(`franka_description_local.patch`, apply with
`cd ~/catkin_ws/src/franka_ros && git apply <path>/franka_description_local.patch`).
Restart the ROS launch (franka_control.launch) afterwards.

| What | Stock | This robot | Source |
|---|---|---|---|
| flange (panda_link8) -> panda_hand | z 0 | **z 0.02** (20 mm adapter) | measured by the user, 2026-10-12 |
| flange -> panda_hand_tcp | 0.1034 | **0.1234** | follows from the above (hand -> TCP unchanged) |
| finger tip collision | rubber tip box 17.5 x 15.2 x 18.5 mm, finger z 36-54.5 mm | foam + printed block + metal 15 x 11 mm (finger z 43.6-54.6), wedge tip 20 mm high narrowing 12 -> 2 mm (two stepped boxes, to z 74.6 = 67 mm below the hand body), metal bar 3 mm above the block | all measured by the user 2026-10-12 (sketch + 11 mm, 67 mm, opening 80 mm at full open = grasp face at the stock inner face); width 25 mm assumed |

Not in the URDF (attached at API startup, see python/restful.py):
- `wrist_camera`: D435 box + 3D-printed mount, anchored to the flange in
  pipeFit's nominal TCP frame (link8 * Rz(-45 deg) * z 0.1034).
- `flange_adapter`: cylinder r 0.04 (ESTIMATED), length 0.02 between flange and hand.

Consequences for clients: the TCP (panda_hand_tcp, hand + 0.1034) the API
reports and accepts is now 20 mm lower than before, matching the real hand: the
same commanded TCP target puts the hand 20 mm higher than before. The foam pad
centre is ~4 mm below the TCP; the fingertips reach ~29 mm below it. The
reported gripper width equals the real gap between the foam pads. Franka Desk's end-effector settings
(F_T_EE z = 0.1034, mass 0.73 kg) still describe a stock hand.
