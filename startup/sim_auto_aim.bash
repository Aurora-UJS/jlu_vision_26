#!/usr/bin/bash
# Bring up the auto-aim pipeline against the Webots simulator, natively.
#
# The simulator publishes frames on /aurora_rm_vision and exchanges gimbal
# angles on /aurora_rm_aim; start it first, e.g.
#
#   RM_SIM_INITIAL_YAW=1.5708 RM_SIM_INITIAL_PITCH=-0.1276 \
#   RM_SIM_AIM_ALWAYS_ENGAGED=1 webots worlds/auto_aim_bench.wbt
#
# Pass --viz to turn on the detector's own result windows.
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
prefix="${RM_SIM_PREFIX:-${HOME}/.local/rmsim}"
bin="${repo_dir}/build/linux/x86_64/release"
visualise=0
[[ "${1:-}" == "--viz" ]] && visualise=1

# iceoryx and gtsam live under the prefix; the vendor camera SDKs and OpenVINO
# are only needed at load time, not by the simulator backends themselves.
export LD_LIBRARY_PATH="${prefix}/lib:\
${repo_dir}/src/third_party/galaxy_camera_sdk/lib/amd64:\
${repo_dir}/src/third_party/hik_camera_sdk/lib/amd64:\
/opt/intel/openvino_2024/runtime/lib/intel64:${LD_LIBRARY_PATH:-}"

cd "${repo_dir}"
mkdir -p logs/camera logs/serial logs/detector

# A node that was killed leaves its name registered until RouDi times it out,
# and re-registering the same name is fatal.  Starting from a clean RouDi is
# quicker than waiting that out.
# -x matches the process name, not the command line.  A -f pattern would also
# match this script, which mentions those names, so the script would kill
# itself before ever starting anything.
# One pattern per call: pkill takes a single pattern, and extra arguments make
# it fail with a usage error -- silently killing nothing behind the || true.
for node in camera serial armor_detector armor_tracker static_tf_bc \
            sim_viewer iox-roudi; do
  pkill -x "${node}" 2>/dev/null || true
done
sleep 2
rm -f /dev/shm/iox1_* 2>/dev/null || true

"${prefix}/bin/iox-roudi" -c configs/iox-roudi_config.toml >/tmp/rm_sim_roudi.log 2>&1 &
for _ in $(seq 40); do
  grep -q "RouDi is ready for clients" /tmp/rm_sim_roudi.log && break
  sleep 0.25
done

"${bin}/camera" -c configs/hardware/camera_sim.yaml -l logs/camera \
  >/tmp/rm_sim_camera.log 2>&1 &
"${bin}/serial" -c configs/hardware/serial_sim.yaml -l logs/serial \
  >/tmp/rm_sim_serial.log 2>&1 &
sleep 3

detector_config=configs/auto_aim/armor_detector_sim.yaml
if [[ "${visualise}" == 1 ]]; then
  detector_config=/tmp/rm_sim_detector_viz.yaml
  sed -e 's/^show_detect_result: false$/show_detect_result: true/' \
      -e 's/^show_optimize_result: false$/show_optimize_result: true/' \
      -e 's/^show_pnp_result: false$/show_pnp_result: true/' \
      configs/auto_aim/armor_detector_sim.yaml >"${detector_config}"
fi
"${bin}/armor_detector" -c "${detector_config}" -l logs/detector \
  >/tmp/rm_sim_detector.log 2>&1 &
"${bin}/static_tf_bc" -c configs/odom_coord/static_tf_bc_sim.yaml -l logs/tf \
  >/tmp/rm_sim_tf.log 2>&1 &
sleep 3
"${bin}/armor_tracker" -c configs/auto_aim/armor_tracker_sim.yaml -l logs/tracker \
  >/tmp/rm_sim_tracker.log 2>&1 &

sleep 5
echo "roudi/camera/serial/detector/tf/tracker up; logs in /tmp/rm_sim_*.log"
[[ "${visualise}" == 1 ]] && echo "detector windows enabled"
wait
