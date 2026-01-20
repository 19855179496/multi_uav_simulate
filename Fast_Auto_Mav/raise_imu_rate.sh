#!/bin/sh
# raise_imu_rate.sh
# Usage: sh raise_imu_rate.sh <count>

COUNT=${1:-5}
# validate numeric
case "$COUNT" in
  ''|*[!0-9]*)
    echo "Invalid count: $COUNT" >&2
    exit 1
    ;;
esac

i=0
while [ "$i" -lt "$COUNT" ]; do
  ns="uav${i}"
  svc="/${ns}/mavros/cmd/command"
  echo "[raise_imu_rate] waiting for service ${svc} ..."
  # wait until the mavros service appears
  while ! rosservice list 2>/dev/null | grep -xq "${svc}"; do
    sleep 0.5
  done

  echo "[raise_imu_rate] sending set interval to ${ns} (msg 105 -> 5000us, 31 -> 5000us, 32 -> 10000us)"
  rosservice call "${svc}" "{broadcast: false, command: 511, confirmation: 0, param1: 105, param2: 5000, param3: 0, param4: 0, param5: 0, param6: 0, param7: 0}" || echo "[warn] call failed for ${ns} msg 105"
  sleep 0.2
  rosservice call "${svc}" "{broadcast: false, command: 511, confirmation: 0, param1: 31, param2: 5000, param3: 0, param4: 0, param5: 0, param6: 0, param7: 0}" || echo "[warn] call failed for ${ns} msg 31"
  sleep 0.2
  rosservice call "${svc}" "{broadcast: false, command: 511, confirmation: 0, param1: 32, param2: 10000, param3: 0, param4: 0, param5: 0, param6: 0, param7: 0}" || echo "[warn] call failed for ${ns} msg 32"

  i=$((i+1))
done

echo "[raise_imu_rate] done for ${COUNT} UAV(s)."
