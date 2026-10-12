#!/bin/bash
# P2 (colours, cleanup): restart -> phase1 -> restart -> phase2. Sim only.
HERE="$(cd "$(dirname "$0")" && pwd)"; source "$HERE/env.sh"
wait_up() { for i in $(seq 1 60); do curl -s -m 1 http://127.0.0.1:5100/scene/info | grep -q scene_version && return 0; grep -q Traceback "$SIM_DIR/server.log" && { grep -A20 Traceback "$SIM_DIR/server.log" | head -25; return 1; }; sleep 2; done; return 1; }
"$HERE/restart_server.sh" >/dev/null 2>&1 && wait_up || exit 1
echo "===== P2 phase 1"; python3 "$HERE/test_p2.py" phase1 2>&1 | grep -E "^(PASS|FAIL)|passed|Traceback|Error|File \""
"$HERE/restart_server.sh" >/dev/null 2>&1 && wait_up || exit 1
echo "===== P2 phase 2 (after restart)"; python3 "$HERE/test_p2.py" phase2 2>&1 | grep -E "^(PASS|FAIL)|passed|Traceback|Error|File \""
