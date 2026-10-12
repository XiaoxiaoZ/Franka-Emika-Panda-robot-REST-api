# Built-in collision models

- `panda_realsense_D4xx.stl` -- 3D-printed wrist mount for the RealSense D435
  (from github.com/dougsm/mvp_grasp, `cad/`), in millimetres. Mounted on the
  -x face of the Franka hand, flat side on the hand, wide end (z = 38 mm)
  carrying the back of the camera, stud end towards the flange (photo
  2026-10-12). Used as its convex hull in the `wrist_camera` built-in object.
  Note: mvp_grasp's own launch file places the camera on the +x side; this
  robot has it on -x, matching pipeFit's hand-eye calibration.
