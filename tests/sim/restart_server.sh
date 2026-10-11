#!/bin/bash
# (Re)start the dev API server against the sim: 127.0.0.1:5100, scratch state/files, test hooks on.
HERE="$(cd "$(dirname "$0")" && pwd)"; source "$HERE/env.sh"
pid=$(ss -ltnp 2>/dev/null | awk '/127.0.0.1:5100 /{print $NF}' | grep -o 'pid=[0-9]*' | head -1 | cut -d= -f2)
if [ -n "$pid" ]; then kill -INT "$pid"; for i in $(seq 1 20); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done; kill -9 "$pid" 2>/dev/null; fi
export FRANKA_RESTFUL_HOST=127.0.0.1 FRANKA_RESTFUL_PORT=5100 FRANKA_TEST_HOOKS=1
export FRANKA_STATE_DIR="$SIM_DIR/state" FRANKA_FILES_ROOT="$SIM_DIR/files"
cd "$HERE/../../python" && setsid nohup python3 restful.py > "$SIM_DIR/server.log" 2>&1 < /dev/null &
echo "dev server restarting (old pid ${pid:-none}); log $SIM_DIR/server.log"
