# Scene/plan API acceptance tests (simulation)

Runs the requirements' acceptance tests T1-T17, T20, T21 (plus hash, S1, S2,
Q2 checks) against MoveIt's `demo.launch` with fake controllers on a separate
ROS master (port 11312) and a dev API server on `127.0.0.1:5100`. Nothing here
talks to the real robot's master or server.

    ./start_sim.sh            # once; wait for "You can start planning now!"
    ./run_all.sh              # all suites, restarting the dev server in between (~10-12 min)
    ./p1_only.sh              # just the P1 suite (~4 min)

Scratch data (state, files, logs) goes to `$SIM_DIR` (default /tmp/franka_sim).
`test_p1.py` covers P1 (T18 attach/detach, T19 ACM, padding, /robot/tf, /plan/ik,
check_trajectory, dry_run_sequence, restart persistence); `p1_only.sh` runs just those.
`test_p2.py` / `p2_only.sh`: object colours (as RViz receives them) and file cleanup.
