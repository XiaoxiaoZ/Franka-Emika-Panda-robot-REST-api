#!/bin/bash
# restart -> scene suite (T1-T11, T20) -> restart -> plan suite (T12-T17, T21). Sim only.
HERE="$(cd "$(dirname "$0")" && pwd)"; source "$HERE/env.sh"
wait_up() { for i in $(seq 1 60); do curl -s -m 1 http://127.0.0.1:5100/scene/info | grep -q scene_version && return 0; grep -q Traceback "$SIM_DIR/server.log" && { grep -A20 Traceback "$SIM_DIR/server.log" | head -25; return 1; }; sleep 2; done; return 1; }
"$HERE/restart_server.sh" >/dev/null && wait_up || exit 1
echo "===== scene suite"; python3 "$HERE/test_scene_p0.py" 2>&1 | grep -E "^(PASS|FAIL)|passed"
"$HERE/restart_server.sh" >/dev/null && wait_up || exit 1
echo "===== plan suite (after restart)"; python3 "$HERE/test_plan_p0.py" 2>&1 | grep -E "^(PASS|FAIL)|passed"
