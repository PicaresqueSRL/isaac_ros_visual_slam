#!/bin/bash
# From the Mac (or any machine with ssh): starts pq_console on the remote machine, opens an ssh
# tunnel to it and the page in the browser. Ctrl-C closes the tunnel (the console keeps running).
# Usage: connect.sh <user@host> [remote repo folder] [local port]
HOST=${1:?user@host}; REPO=${2:-workspaces/pq_cuvslam/isaac_ros_visual_slam}; PORT=${3:-8080}
ssh "$HOST" "$REPO/pq_console/run.sh start" || exit 1
URL=http://localhost:$PORT
( sleep 2; command -v open > /dev/null && open "$URL" || xdg-open "$URL" > /dev/null 2>&1 ) &
echo "pq_console: $URL (Ctrl-C closes the tunnel)"
exec ssh -N -L "$PORT:127.0.0.1:8080" "$HOST"
