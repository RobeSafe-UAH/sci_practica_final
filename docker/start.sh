#!/usr/bin/env bash
set -eo pipefail

# ROS setup scripts are sourced before nounset because they inspect optional vars.
source /opt/ros/jazzy/setup.bash
source /opt/sci_ws/install/setup.bash
set -u

if [[ "${1:-simulate}" != simulate ]]; then
    exec "$@"
fi
if (( $# > 1 )); then
    echo 'simulate takes its settings from SCI_* environment variables.' >&2
    exit 2
fi

case "${SCI_HEADLESS,,}" in
    true|false) SCI_HEADLESS="${SCI_HEADLESS,,}" ;;
    *) echo 'SCI_HEADLESS must be true or false.' >&2; exit 2 ;;
esac
for variable in SCI_VNC_PORT SCI_SCREEN_WIDTH SCI_SCREEN_HEIGHT; do
    value="${!variable}"
    if [[ ! "$value" =~ ^[0-9]+$ ]] || (( 10#$value < 1 || 10#$value > 65535 )); then
        echo "$variable must be an integer between 1 and 65535." >&2
        exit 2
    fi
done
if [[ ! -r "$SCI_CONFIG_FILE" ]]; then
    echo "Scenario configuration is not readable: $SCI_CONFIG_FILE" >&2
    exit 2
fi

runtime_dir=/tmp/sci-runtime
umask 077
mkdir -p "$runtime_dir"
chmod 0700 "$runtime_dir"
rm -f "$runtime_dir/state.json"
export XDG_RUNTIME_DIR="$runtime_dir"
export XAUTHORITY="$runtime_dir/Xauthority"
touch "$XAUTHORITY"
cookie="$(python3 -c 'import secrets; print(secrets.token_hex(16))')"
# The server loads the cookie independently of the display number in this
# private file. Add the client entry for the selected display after startup.
xauth -f "$XAUTHORITY" add :0 MIT-MAGIC-COOKIE-1 "$cookie"

pids=()
declare -A process_names=()
cleanup() {
    trap - EXIT INT TERM
    rm -f "$runtime_dir/state.json"
    for pid in "${pids[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || true
    done
    for (( attempt=0; attempt<50; attempt++ )); do
        running=false
        for pid in "${pids[@]}"; do
            if kill -0 -- "-$pid" 2>/dev/null; then running=true; fi
        done
        if [[ "$running" == false ]]; then break; fi
        sleep 0.1
    done
    for pid in "${pids[@]}"; do
        kill -KILL -- "-$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_process() {
    local name="$1"
    shift
    # Each service gets a process group so cleanup includes its descendants.
    setsid "$@" &
    last_pid=$!
    pids+=("$last_pid")
    process_names["$last_pid"]="$name"
}

wait_for() {
    local name="$1" pid="$2" deadline=$(( SECONDS + 30 ))
    shift 2
    until "$@" >/dev/null 2>&1; do
        if ! kill -0 "$pid" 2>/dev/null; then
            echo "$name exited before becoming ready." >&2
            return 1
        fi
        if (( SECONDS >= deadline )); then
            echo "Timed out waiting for $name." >&2
            return 1
        fi
        sleep 0.1
    done
}

# Check the configurable web port before starting anything. The VNC backend
# uses an ephemeral loopback port so an existing host VNC server is unaffected.
vnc_port="$(python3 - "$SCI_VNC_PORT" <<'PY'
import errno
import socket
import sys

port = int(sys.argv[1])
with socket.socket() as web, socket.socket() as vnc:
    # Match the web server's reuse policy: TIME_WAIT connections from a
    # previous session must not prevent an immediate restart.
    web.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        web.bind(('127.0.0.1', port))
    except OSError as error:
        if error.errno != errno.EADDRINUSE:
            raise
        alternative = port + 1 if port < 65535 else 6080
        sys.exit(
            f'SCI: el puerto web {port} ya está en uso.\n'
            f'Detén el servicio que lo ocupa o ejecuta: SCI_VNC_PORT={alternative} make run\n'
            f'Navegador con el puerto alternativo: http://localhost:{alternative}/vnc.html'
        )
    # Hold the web reservation while choosing VNC so the ports cannot coincide.
    vnc.bind(('127.0.0.1', 0))
    print(vnc.getsockname()[1])
PY
)"

# Let Xvfb atomically reserve an available display, including its abstract
# socket in the shared network namespace. DISPLAY's unix/ prefix does not
# prevent Xlib from trying abstract sockets, so both transports stay enabled.
display_file="$runtime_dir/display"
: > "$display_file"
exec {display_fd}>"$display_file"
start_process xvfb Xvfb -displayfd "$display_fd" \
    -screen 0 "${SCI_SCREEN_WIDTH}x${SCI_SCREEN_HEIGHT}x24" \
    -nolisten tcp -auth "$XAUTHORITY" -noreset
xvfb_pid="$last_pid"
exec {display_fd}>&-
wait_for Xvfb "$xvfb_pid" test -s "$display_file"
read -r display_number < "$display_file"
if [[ ! "$display_number" =~ ^[0-9]+$ ]]; then
    echo "Xvfb returned an invalid display number: $display_number" >&2
    exit 1
fi
export DISPLAY=":$display_number"
xauth -f "$XAUTHORITY" add "$DISPLAY" MIT-MAGIC-COOKIE-1 "$cookie"
if [[ "$display_number" != 0 ]]; then
    xauth -f "$XAUTHORITY" remove :0
fi
unset cookie
wait_for Xvfb "$xvfb_pid" xdpyinfo -display "$DISPLAY"

start_process fluxbox fluxbox -display "$DISPLAY"
start_process x11vnc x11vnc -display "$DISPLAY" -auth "$XAUTHORITY" \
    -listen 127.0.0.1 -localhost -rfbport "$vnc_port" \
    -forever -shared -nopw -noxdamage
wait_for x11vnc "$last_pid" python3 /opt/sci/docker/healthcheck.py --vnc-port "$vnc_port"

start_process websockify websockify --web=/usr/share/novnc \
    "127.0.0.1:$SCI_VNC_PORT" "127.0.0.1:$vnc_port"
wait_for noVNC "$last_pid" python3 /opt/sci/docker/healthcheck.py --http-port "$SCI_VNC_PORT"

start_process simulation ros2 launch sci_mvsim simulator.launch.py \
    "config_file:=$SCI_CONFIG_FILE" "headless:=$SCI_HEADLESS"

state_args=()
for pid in "${pids[@]}"; do
    state_args+=("${process_names[$pid]}:$pid")
done
python3 - "$runtime_dir/state.json" "$SCI_VNC_PORT" "$vnc_port" \
    "$DISPLAY" "$XAUTHORITY" "${state_args[@]}" <<'PY'
import json
from pathlib import Path
import sys

target = Path(sys.argv[1])
processes = dict(item.split(':', 1) for item in sys.argv[6:])
state = {
    'web_port': int(sys.argv[2]),
    'vnc_port': int(sys.argv[3]),
    'display': sys.argv[4],
    'xauthority': sys.argv[5],
    'processes': {name: int(pid) for name, pid in processes.items()},
}
temporary = target.with_suffix('.tmp')
temporary.write_text(json.dumps(state))
temporary.replace(target)
PY

echo "SCI desktop: http://localhost:$SCI_VNC_PORT/vnc.html"
echo "Virtual X11 display: $DISPLAY"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}; headless=$SCI_HEADLESS"
# GUI health is separate from the ROS sensor acceptance checks.
exit_code=0
finished_pid=''
wait -n -p finished_pid "${pids[@]}" || exit_code=$?
echo "SCI service ${process_names[${finished_pid:-0}]:-unknown} exited (status $exit_code)." >&2
if (( exit_code == 0 )); then exit_code=1; fi
exit "$exit_code"
