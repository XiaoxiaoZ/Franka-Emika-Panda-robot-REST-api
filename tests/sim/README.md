# Scene/plan API acceptance tests (simulation)

Runs the requirements' acceptance tests T1-T17, T20, T21 (plus hash, S1, S2,
Q2 checks) against MoveIt's `demo.launch` with fake controllers on a separate
ROS master (port 11312) and a dev API server on `127.0.0.1:5100`. Nothing here
talks to the real robot's master or server.

    ./start_sim.sh            # once; wait for "You can start planning now!"
    ./run_all.sh              # restarts the dev server between the suites (T21)

Scratch data (state, files, logs) goes to `$SIM_DIR` (default /tmp/franka_sim).
T18/T19 (v2 attach, ACM) belong to P1 and are not covered yet.
