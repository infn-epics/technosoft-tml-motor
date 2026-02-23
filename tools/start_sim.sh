#!/bin/bash
#
# start_sim.sh — Start the TML drive simulator with a virtual serial pair.
#
# Creates a socat virtual serial pair:
#   /var/tmp/ttyV0  ←→  /var/tmp/ttyV1
#
# The IOC connects to /var/tmp/ttyV0 (configured in singleAxis.cmd).
# The simulator listens on /var/tmp/ttyV1.
#
# Usage:
#   ./start_sim.sh          # foreground, verbose
#   ./start_sim.sh --bg     # background (simulator + socat)
#   ./start_sim.sh --stop   # kill background processes
#
# Author: Andrea Michelotti — INFN-LNF (2026-02)
#

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TOP="$(cd "$SCRIPT_DIR/.." && pwd)"
SIM_PY="$SCRIPT_DIR/tml_sim.py"

TTY_IOC="/var/tmp/ttyV0"
TTY_SIM="/var/tmp/ttyV1"
AXIS_ID=15

PID_SOCAT="/var/tmp/tml_socat.pid"
PID_SIM="/var/tmp/tml_sim.pid"

# --- Stop mode ---
if [[ "$1" == "--stop" ]]; then
    echo "Stopping TML simulator..."
    [[ -f "$PID_SIM"   ]] && kill "$(cat "$PID_SIM")" 2>/dev/null && rm -f "$PID_SIM"
    [[ -f "$PID_SOCAT" ]] && kill "$(cat "$PID_SOCAT")" 2>/dev/null && rm -f "$PID_SOCAT"
    rm -f "$TTY_IOC" "$TTY_SIM"
    echo "Done."
    exit 0
fi

# --- Check dependencies ---
if ! command -v socat &>/dev/null; then
    echo "ERROR: socat not found. Install with: apt-get install socat"
    exit 1
fi
if ! command -v python3 &>/dev/null; then
    echo "ERROR: python3 not found."
    exit 1
fi

# --- Clean stale links ---
rm -f "$TTY_IOC" "$TTY_SIM"

# --- Start socat virtual serial pair ---
echo "Starting socat virtual serial pair: $TTY_IOC <-> $TTY_SIM"
socat -d -d \
    "pty,raw,echo=0,link=$TTY_IOC" \
    "pty,raw,echo=0,link=$TTY_SIM" &
SOCAT_PID=$!
echo "$SOCAT_PID" > "$PID_SOCAT"

# Wait for the pty links to appear
for i in $(seq 1 30); do
    [[ -L "$TTY_IOC" && -L "$TTY_SIM" ]] && break
    sleep 0.1
done

if [[ ! -L "$TTY_IOC" || ! -L "$TTY_SIM" ]]; then
    echo "ERROR: socat failed to create pty links"
    kill "$SOCAT_PID" 2>/dev/null
    exit 1
fi

echo "Virtual serial pair ready."
sleep 0.2

# --- Start simulator ---
if [[ "$1" == "--bg" ]]; then
    echo "Starting TML simulator (background) on $TTY_SIM axis=$AXIS_ID"
    python3 "$SIM_PY" "$TTY_SIM" "$AXIS_ID" &
    SIM_PID=$!
    echo "$SIM_PID" > "$PID_SIM"
    echo ""
    echo "=== TML Simulation Running ==="
    echo "  socat PID: $SOCAT_PID"
    echo "  sim   PID: $SIM_PID"
    echo "  IOC port:  $TTY_IOC"
    echo ""
    echo "To stop:  $0 --stop"
    echo "To start IOC:"
    echo "  cd $TOP/iocBoot/iocTml"
    echo "  ../../bin/linux-x86_64/technosoft ./st.cmd"
else
    echo "Starting TML simulator (foreground) on $TTY_SIM axis=$AXIS_ID"
    echo "Press Ctrl+C to stop."
    echo ""
    python3 "$SIM_PY" "$TTY_SIM" "$AXIS_ID"
    # Clean up socat when simulator exits
    kill "$SOCAT_PID" 2>/dev/null
    rm -f "$PID_SOCAT" "$PID_SIM"
fi
