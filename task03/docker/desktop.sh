#!/usr/bin/env bash
set -euo pipefail
export DISPLAY="${DISPLAY:-:99}"
export LIBGL_ALWAYS_SOFTWARE=1
children=()
cleanup() {
  trap - TERM INT EXIT
  for child in "${children[@]}"; do kill "$child" 2>/dev/null || true; done
  wait || true
}
trap cleanup TERM INT EXIT
Xvfb "$DISPLAY" -screen 0 1440x900x24 -ac -nolisten tcp &
children+=("$!")
for attempt in $(seq 1 50); do
  if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then break; fi
  sleep 0.1
done
xdpyinfo -display "$DISPLAY" >/dev/null
openbox > /tmp/course-openbox.log 2>&1 &
children+=("$!")
x11vnc -display "$DISPLAY" -forever -shared -nopw -localhost -rfbport 5900 > /tmp/course-vnc.log 2>&1 &
children+=("$!")
websockify --web=/usr/share/novnc/ 6080 localhost:5900 > /tmp/course-websockify.log 2>&1 &
children+=("$!")
wait -n "${children[@]}"
