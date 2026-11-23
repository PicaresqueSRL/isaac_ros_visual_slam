#!/bin/bash
# Starts pq_console in a container (detached; it keeps running when the ssh session ends).
# The console drives the other containers through the Docker socket, with the docker client of
# the host mounted read-only. It listens on the address of the configuration (default
# 127.0.0.1:8080): reach it through an ssh tunnel (connect.sh).
# Usage: run.sh [start|restart|stop|logs] [config.yaml]   (default: start, config.yaml next to this script)
set -e
DIR=$(cd "$(dirname "$0")" && pwd)
CMD=${1:-start}; CONFIG=$(realpath "${2:-$DIR/config.yaml}")
key() { sed -n "s/^$1: *//p" "$CONFIG" | sed 's/ *#.*//; s/^["'\'']//; s/["'\'']$//' | head -1; }
case $CMD in
  stop) docker rm -f pq_console > /dev/null 2>&1 || true; echo "pq_console stopped"; exit 0 ;;
  logs) exec docker logs -f pq_console ;;
  restart) docker rm -f pq_console > /dev/null 2>&1 || true ;;
  start) ;;
  *) echo "usage: run.sh [start|restart|stop|logs] [config.yaml]"; exit 1 ;;
esac
[ -f "$CONFIG" ] || { echo "no configuration $CONFIG (copy config.example.yaml)"; exit 1; }
if docker ps -q --filter name=^pq_console$ | grep -q .; then
  echo "pq_console already running (run.sh restart to restart it)"; exit 0
fi
DATA=$(key data_dir); BAGS=$(key bags_dir); IMAGE=$(key slam_image); IMAGE=${IMAGE:-pq_cuvslam:dev}
DOMAIN=$(key ros_domain_id); DDS=$(key dds_transports)
[ -n "$DATA" ] || { echo "data_dir missing in $CONFIG"; exit 1; }
mkdir -p "$DATA/bags" "$DATA/runs" "$DATA/rerun"
VOLS=(-v "$DATA:$DATA")
[ -n "$BAGS" ] && VOLS+=(-v "$BAGS:$BAGS")
INSTALL=$(key install_dir); [ -n "$INSTALL" ] && VOLS+=(-v "$INSTALL:/ws/install:ro")
docker rm -f pq_console > /dev/null 2>&1 || true
docker run -d --name pq_console --network host --ipc host \
  -e ROS_DOMAIN_ID=${DOMAIN:-7} -e FASTDDS_BUILTIN_TRANSPORTS=${DDS:-LARGE_DATA} \
  -e HOST_UID=$(id -u) -e HOST_GID=$(id -g) \
  -v /var/run/docker.sock:/var/run/docker.sock -v "$(command -v docker)":/usr/bin/docker:ro \
  -v "$DIR":/console:ro -v "$CONFIG":/console_config.yaml:ro "${VOLS[@]}" \
  --entrypoint bash "$IMAGE" -c "source /opt/ros/lyrical/setup.bash; source /ws/install/setup.bash; \
  exec python3 -u /console/console.py --config /console_config.yaml" > /dev/null
sleep 3
docker logs pq_console 2>&1 | tail -3
