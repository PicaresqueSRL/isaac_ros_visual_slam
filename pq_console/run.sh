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
IMAGE=$(key slam_image); IMAGE=${IMAGE:-pq_cuvslam:dev}
# Folders of the configuration, mounted at the same path: "rw <path>" or "ro <path>".
MOUNTS=$(docker run --rm --network none -v "$CONFIG:/c.yaml:ro" --entrypoint python3 "$IMAGE" -c '
import os, yaml
c = yaml.safe_load(open("/c.yaml")) or {}
d = c.get("data_dir") or ""
if not d: raise SystemExit("data_dir missing")
print("rw", d); print("rw", c.get("bags_dir") or os.path.join(d, "bags"))
for r in c.get("browse_roots") or []: print("ro", r)
if c.get("install_dir"): print("install", c["install_dir"])
print("domain", c.get("ros_domain_id", 7)); print("dds", c.get("dds_transports", "LARGE_DATA"))
') || { echo "cannot read $CONFIG"; exit 1; }
VOLS=(); DOMAIN=7; DDS=LARGE_DATA
while read -r kind path; do
  case $kind in
    rw) mkdir -p "$path"; VOLS+=(-v "$path:$path") ;;
    ro) [ -d "$path" ] && VOLS+=(-v "$path:$path:ro") ;;
    install) VOLS+=(-v "$path:/ws/install:ro") ;;
    domain) DOMAIN=$path ;;
    dds) DDS=$path ;;
  esac
done <<< "$MOUNTS"
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
