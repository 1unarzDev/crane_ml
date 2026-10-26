#!/usr/bin/env bash
# Headless crane_ml run against the ROS stack in the roboboat_dev container.
#
# The player runs on the host (batchmode, Vulkan on an X display) and talks to a
# ROS-TCP endpoint in the container. The container also runs the profile's
# localization (the profile's start: components, or its older bringup:), thruster_pwm_node with
# the topic backend, a rosbag2 recorder and a drive (or --run command). Every
# container process uses its own ROS domain and the localhost discovery range.
#
# Usage: run_sim.sh [options] [-- extra player args]
#   -c, --course NAME|PATH   course spec: a name in courses_dir or a .json path [roboboat_2025]
#   -f, --faults PATH        calibration faults JSON (--crane-calibration-faults) [none]
#   -b, --boat-profile NAME|PATH.json  boat plant: ideal, characterized or an asset name
#                            (--crane-boat-profile), or a profile JSON (--crane-boat-profile-json)
#                            [ideal]; the player's CRANE_PLANT_APPLIED line goes to the run log
#   -r, --robot NAME         robot in place of the scene's boat: a URDF robot name from the
#                            player's RobotCatalog (--crane-robot) [the scene's]
#   -d, --duration S         seconds recorded after the course is ready [30]
#   -p, --profile NAME|PATH  mhseals_testing profile [crane_full_stack]
#   -o, --out DIR            session dir [sessions_dir/<profile>_<YYYYmmdd_HHMMSS>]
#       --current S,DEG      water current: m/s toward DEG, CCW from world +x
#       --wind S,DEG         mean wind (gusts seeded by --seed)
#       --gps-sigma M        GPS noise standard deviation (crane --crane-gps-sigma)
#       --seed N             --crane-seed and --crane-contract-seed [rig.yaml seed]
#       --mode effort|velocity  driver chain: thruster_pwm_node on the command topic, or the
#                            harness velocity chain (controller, mux) as boat_test arms it [rig.yaml]
#       --hardware PATH      hardware YAML for the driver chain [the install's config/sim/hardware_crane.yaml]
#       --task NAME          after the run: events.jsonl, referee.json (judge_task over the course
#                            truth and /crane/odom) and the task outcome in session.yaml
#       --run CMD            shell command (bash -c) instead of the default drive; it
#                            sees SIMRIG_DIR (its session dir in the container) and
#                            ROS_DOMAIN_ID, and is cut off at the duration (the recording
#                            ends early when it exits); the session exits 1 when it fails
#       --drive NAME         a drive preset from rig.yaml drives: (topic and schedule) instead
#                            of the default drive
#       --no-drive           record only
#       --overlay PATH       container install whose local_setup.bash is sourced after the base install
#       --domain N           ROS_DOMAIN_ID; one run per domain and per port (host lock files)
#       --port N             ROS-TCP endpoint port
#       --smoke              gate the bag with check.py; exit 1 when a gate fails
#       --config PATH        rig YAML [rig.yaml next to this script], merged with rig.local.yaml
#                            (or $CRANE_RIG_LOCAL) for this host's paths; see rigconf.py
#       --sim-profile NAME   crane --crane-sim-profile: full|balanced|minimal [full]; the rig never
#                            auto-benchmarks, so runs stay reproducible
#   -h, --help
#
# The session dir gets player.log, the stack logs, bag/, course_truth.json,
# session.yaml (provenance: seed, player build and argv, sha256 of faults, hardware and
# nav2_params, source commit of each package, outcome), rig.json (the merged config),
# run.env (runner and player PIDs, container dir: what rigq's watchdog stops), progress.json
# (probe.py: sim time, topic counts, travel) and (with --smoke) smoke.json.

set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
config="${here}/rig.yaml"
course="" faults="" boat_profile="" robot="" duration="" profile="" out="" current="" run_cmd=""
overlay="" domain="" port="" drive=1 drive_preset="" smoke=0 seed="" wind="" gps_sigma="" sim_profile=full mode="" hardware="" task=""
extra=()

usage() { sed -n '2,/^$/s/^# \{0,1\}//p' "$0"; exit "${1:-0}"; }

while (($#)); do
    case "$1" in
        -c|--course) course="$2"; shift 2 ;;
        -f|--faults) faults="$2"; shift 2 ;;
        -b|--boat-profile) boat_profile="$2"; shift 2 ;;
        -r|--robot) robot="$2"; shift 2 ;;
        -d|--duration) duration="$2"; shift 2 ;;
        -p|--profile) profile="$2"; shift 2 ;;
        -o|--out) out="$2"; shift 2 ;;
        --current) current="$2"; shift 2 ;;
        --wind) wind="$2"; shift 2 ;;
        --gps-sigma) gps_sigma="$2"; shift 2 ;;
        --sim-profile) sim_profile="$2"; shift 2 ;;
        --seed) seed="$2"; shift 2 ;;
        --mode) mode="$2"; shift 2 ;;
        --hardware) hardware="$2"; shift 2 ;;
        --task) task="$2"; shift 2 ;;
        --run) run_cmd="$2"; shift 2 ;;
        --no-drive) drive=0; shift ;;
        --drive) drive_preset="$2"; shift 2 ;;
        --overlay) overlay="$2"; shift 2 ;;
        --domain) domain="$2"; shift 2 ;;
        --port) port="$2"; shift 2 ;;
        --smoke) smoke=1; shift ;;
        --config) config="$2"; shift 2 ;;
        -h|--help) usage ;;
        --) shift; extra=("$@"); break ;;
        *) echo "unknown option $1" >&2; usage 2 ;;
    esac
done

# --- configuration: rig.yaml, the profile and the flags, resolved in one place --------

eval "$(python3 -I - "$here" "$config" "$course" "$profile" "$domain" "$port" "$duration" "$seed" "$mode" "$drive_preset" <<'EOF'
import json, re, shlex, sys, yaml
from pathlib import Path

here, config, course, profile, domain, port, duration, seed, mode, preset = sys.argv[1:]
sys.path.insert(0, here)
import rigconf
rig = rigconf.load(config)
for key in ('player', 'courses_dir', 'profiles_dir', 'container'):
    if not rig.get(key):
        print(f'rig config has no {key}: set it in {rigconf.local_path(config)} (see rig.local.example.yaml)',
              file=sys.stderr)
        print('exit 2')
        sys.exit()
if preset and preset not in (rig.get('drives') or {}):
    print(f'no drive preset {preset!r} in rig.yaml drives: {sorted(rig.get("drives") or {})}', file=sys.stderr)
    print('exit 2')
    sys.exit()
course = course or rig['course']
course = Path(course) if course.endswith('.json') else Path(rig['courses_dir']) / f'{course}.json'
profile = profile or rig['profile']
profile = Path(profile) if profile.endswith('.yaml') else Path(rig['profiles_dir']) / f'{profile}.yaml'
plant = yaml.safe_load(profile.read_text())
if not rig.get('install'):
    print('rig.yaml install is empty: set install (the stack install path inside the container) in rig.local.yaml', file=sys.stderr)
    print('exit 2')
    sys.exit()
values = {
    'container': rig['container'], 'player': rig['player'], 'display': rig['display'],
    'scene': rig['scene'], 'runtime_profile': rig['runtime_profile'],
    'ready_timeout': rig['ready_timeout'], 'install': rig['install'],
    'sessions_dir': rig['sessions_dir'], 'driver': int(bool(rig['driver'])),
    'course_path': course.resolve(), 'profile_path': profile.resolve(),
    'profile_name': plant['name'], 'command_topic': plant['command']['topic'],
    'use_sim_time': str(bool(plant.get('use_sim_time'))).lower(),
    'domain': domain, 'port': port, 'domain_range': ' '.join(map(str, rig['domains'])),
    'max_instances': rig.get('max_instances', 3), 'run_log': rig['run_log'],
    'duration': duration or rig['duration'],
    'drive_schedule': json.dumps(rig['drives'][preset]['schedule'] if preset else rig['drive']),
    'seed': seed or rig.get('seed', 0), 'mode': mode or rig.get('mode', 'effort'),
    'nice': rig.get('nice', 0), 'hardware_default': rig.get('hardware', 'sim'),
    'lidar_smear': int(bool(rig.get('lidar_smear', False))),
    'lock_dir': rig.get('lock_dir', '/tmp'), 'player_commit': rig.get('player_commit') or '',
    'probe': int(bool(rig.get('probe', True))),
    'probe_config': json.dumps(rig.get('watchdog') or {}),
    'drive_topic': (rig['drives'][preset].get('topic') if preset else None)
                   or rig.get('drive_topic') or plant['command']['topic'],
}

import stack
# The profile's stack (stack.py): an older profile names its launches; a current one is resolved in
# the container once it is up (stack_resolve=1).
names = {} if stack.needs_install(plant) else stack.from_profile(plant)
if not names and not stack.needs_install(plant):
    print(f'profile {profile} starts no stack component', file=sys.stderr)
    print('exit 2')
    sys.exit()
values['stack_resolve'] = int(stack.needs_install(plant))
for key, value in values.items():
    print(f'{key}={shlex.quote(str(value))}')
print('stack_names=(' + ' '.join(names) + ')')
print('stack_cmds=(' + ' '.join(map(shlex.quote, names.values())) + ')')
print('player_args=(' + ' '.join(map(shlex.quote, rig['player_args'])) + ')')
print('stack_check=(' + ' '.join(['endpoint'] + list(names)) + ')')
print('record=(' + ' '.join(map(shlex.quote, rig['record'])) + ')')
EOF
)"

((${#stack_names[@]} || stack_resolve)) || { echo "could not resolve the profile's stack" >&2; exit 2; }
# At most max_instances runs at once on this machine, whoever starts them: wait for a slot.
slot_wait=0
while :; do
    for ((i = 0; i < max_instances; i++)); do
        exec 7>"${lock_dir}/simrig-slot-${i}.lock"
        flock -n 7 && break 2
        exec 7>&-
    done
    ((slot_wait++ % 30)) || echo "waiting for one of ${max_instances} rig slots" >&2
    sleep 2
done
# A free domain (and its port) unless given; the locks below hold both for the whole run.
if [[ -z "${domain}" ]]; then
    for d in ${domain_range}; do
        exec 8>"${lock_dir}/simrig-domain-${d}.lock"
        if flock -n 8; then domain=${d}; domain_held=1; break; fi
        exec 8>&-
    done
    [[ -n "${domain}" ]] || { echo "no free domain in ${domain_range}" >&2; exit 2; }
fi
port="${port:-$((10000 + domain))}"
# Fast DDS cannot derive a port above domain 232, so the whole stack would die quietly.
[[ "${domain}" =~ ^[0-9]+$ ]] && ((10#${domain} <= 232)) ||
    { echo "--domain must be 0..232 (got ${domain})" >&2; exit 2; }
[[ -f "${course_path}" ]] || { echo "no course ${course_path}" >&2; exit 2; }
[[ -x "${player}" ]] || { echo "no player ${player}; see simrig.md to build it" >&2; exit 2; }
[[ -z "${faults}" || -f "${faults}" ]] || { echo "no faults file ${faults}" >&2; exit 2; }
[[ "${mode}" == effort || "${mode}" == velocity ]] || { echo "--mode effort|velocity (got ${mode})" >&2; exit 2; }
[[ "${seed}" =~ ^[0-9]+$ ]] || { echo "--seed must be a non-negative integer (got ${seed})" >&2; exit 2; }
hardware="${hardware:-${hardware_default}}"
[[ "${hardware}" == sim || -f "${hardware}" ]] || { echo "no hardware file ${hardware}" >&2; exit 2; }
# One run per domain and per port: concurrent runs on either would cross-talk.
if [[ -z "${domain_held:-}" ]]; then
    exec 8>"${lock_dir}/simrig-domain-${domain}.lock"
    flock -n 8 || { echo "domain ${domain} is in use by another rig run" >&2; exit 2; }
fi
exec 9>"${lock_dir}/simrig-port-${port}.lock"
flock -n 9 || { echo "port ${port} is in use by another rig run" >&2; exit 2; }
docker inspect -f '{{.State.Running}}' "${container}" 2>/dev/null | grep -q true ||
    { echo "container ${container} is not running (${here}/container.sh up)" >&2; exit 2; }

session="${out:-${sessions_dir}/${profile_name}_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "${session}"
session="$(cd "${session}" && pwd)"
python3 -I "${here}/rigconf.py" "${config}" >"${session}/rig.json"
# Unique per run: batches reuse basenames (seed0), and the reaper kills by SIMRIG_DIR.
cdir="/tmp/simrig/d${domain}_$(basename "${session}")_$$"
player_pid="" hold_pid=""
now_ms() { echo $((${EPOCHREALTIME/./} / 1000)); }
started=$(now_ms)

log() { local t=$(($(now_ms) - started)); printf '[simrig %4d.%01ds] %s\n' $((t / 1000)) $((t % 1000 / 100)) "$*"; }
# What a watchdog needs to stop this run by exact PID and group; rewritten when the player starts.
write_run_env() {
    printf 'runner_pid=%s\nplayer_pid=%s\nplayer=%s\ncontainer=%s\ncdir=%s\ndomain=%s\nport=%s\n' \
        "$$" "${player_pid}" "${player}" "${container}" "${cdir}" "${domain}" "${port}" >"${session}/run.env.tmp"
    mv -f "${session}/run.env.tmp" "${session}/run.env"
}

# --- container processes: each in its own process group, pid file in ${cdir} ----------

ros_env="unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE; export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 BASH_ENV=; source /opt/ros/jazzy/setup.bash; source ${install}/setup.bash;"
# local_setup: only the overlay's own packages, so it cannot re-prepend an older underlay.
[[ -n "${overlay}" ]] && ros_env+=" source ${overlay}/local_setup.bash;"
# The base install wins on the path over a worktree install named only in --run.
if [[ -z "${overlay}" && "${run_cmd}" =~ (/tmp|/dev/shm)/[^[:space:]]*/install ]]; then
    echo "warning: --run uses ${BASH_REMATCH[0]} but no --overlay is given; the container's" \
         "installed packages shadow it (pass --overlay <install>)" >&2
fi
ros_env+=" export ROS_DOMAIN_ID=${domain} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST SIMRIG_DIR=${cdir};"

cexec() { docker exec -e BASH_ENV= "${container}" bash -c "$1"; }

# Start NAME running COMMAND (bash) in the background, logging to ${cdir}/NAME.log.
cstart() {
    # setsid -f forks from a foreground command, so SIGINT stays deliverable
    # (a shell's background jobs start with SIGINT ignored).
    local inner="echo \$\$ >${cdir}/$1.pid; ${ros_env} cd ${cdir}; exec nice -n ${nice} $2"
    docker exec -e BASH_ENV= "${container}" bash -c \
        "setsid -f bash -c $(printf '%q' "${inner}") >${cdir}/$1.log 2>&1 </dev/null"
}

# Kill whatever still carries this session's SIMRIG_DIR, e.g. a --run hook that setsid'd away.
creap() {
    cexec "for sig in TERM KILL; do
               for env in /proc/[0-9]*/environ; do
                   grep -qaz '^SIMRIG_DIR=${cdir}\$' \$env 2>/dev/null || continue
                   pid=\${env#/proc/}; kill -\$sig \${pid%/environ} 2>/dev/null
               done
               [ \$sig = TERM ] && sleep 0.5
           done; true"
}

# Signal the process groups of NAMES with SIG, then wait up to 10 s for all to exit.
cstop() {
    local sig=$1; shift
    cexec "pids=''
           for name in $*; do
               pid=\$(cat ${cdir}/\$name.pid 2>/dev/null) && kill -${sig} -- -\$pid 2>/dev/null && pids+=\" \$pid\"
           done
           for _ in \$(seq 100); do
               alive=''; for pid in \$pids; do kill -0 \$pid 2>/dev/null && alive=1; done
               [ -z \"\$alive\" ] && exit 0; sleep 0.1
           done
           for pid in \$pids; do kill -KILL -- -\$pid 2>/dev/null; done; true"
}

cleanup() {
    local status=$?
    trap - EXIT INT TERM
    [[ -n "${hold_pid}" ]] && kill "${hold_pid}" 2>/dev/null
    if [[ -n "${player_pid}" ]]; then
        kill -TERM -- "-${player_pid}" 2>/dev/null || true
        for _ in $(seq 50); do kill -0 "${player_pid}" 2>/dev/null || break; sleep 0.1; done
        kill -KILL -- "-${player_pid}" 2>/dev/null || true
    fi
    cstop INT run drive recorder probe "${driver_names[@]}" "${stack_names[@]}" endpoint "${smear_names[@]}" || true
    creap || true
    cexec "${ros_env} ros2 daemon stop >/dev/null 2>&1" || true
    docker cp -q "${container}:${cdir}/." "${session}/" 2>/dev/null || true
    cexec "rm -rf ${cdir}" || true
    rm -f "${session}"/*.pid "${session}"/run.env.tmp
    # One line per run in the run log, failed runs included.
    python3 -I - "${session}" "${status}" "${run_log}" <<'PY' || true
import json, re, sys, time, yaml
from pathlib import Path
session, status, log = Path(sys.argv[1]), int(sys.argv[2]), sys.argv[3]


def plant_applied(path):
    """{profile, sha256, status} of the player's CRANE_PLANT_APPLIED line, or None (rigq too)."""
    try:
        for text in path.read_text(errors='replace').splitlines():
            if text.startswith('CRANE_PLANT_APPLIED '):
                return dict(re.findall(r'(\w+)=(.*?)(?= \w+=|$)', text[len('CRANE_PLANT_APPLIED '):]))
    except OSError:
        pass
    return None


meta = {}
try:
    meta = yaml.safe_load((session / 'session.yaml').read_text()) or {}
except (OSError, yaml.YAMLError):
    pass
keys = ('seed', 'mode', 'course', 'faults', 'faults_sha256', 'hardware', 'hardware_sha256', 'current',
        'wind', 'gps_sigma', 'overlay', 'nav2_params_sha256', 'packages', 'player', 'run', 'run_status',
        'task_outcome', 'ros')
line = dict(time=time.strftime('%Y-%m-%dT%H:%M:%S%z'), session=str(session), exit=status,
            bag=str(session / 'bag'), **{k: meta.get(k) for k in keys})
line['plant'] = plant_applied(session / 'player.log')
referee = session / 'referee.json'
if referee.exists():
    line['referee'] = json.loads(referee.read_text()).get('verdict')
with open(log, 'a') as out:
    out.write(json.dumps(line, default=str) + '\n')
PY
    log "session ${session} (exit ${status})"
    exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
write_run_env

cexec "rm -rf ${cdir}; mkdir -p ${cdir}"
docker cp -q "${here}/drive.py" "${container}:${cdir}/drive.py"
docker cp -q "${here}/check.py" "${container}:${cdir}/check.py"
docker cp -q "${session}/rig.json" "${container}:${cdir}/rig.yaml"
docker cp -q "${here}/probe.py" "${container}:${cdir}/probe.py"
docker cp -q "${here}/post.py" "${container}:${cdir}/post.py"
if [[ "${hardware}" == sim ]]; then
    cexec "${ros_env} cp \$(ros2 pkg prefix mhseals_testing)/share/mhseals_testing/config/sim/hardware_crane.yaml ${cdir}/hardware.yaml"
else
    docker cp -q "${hardware}" "${container}:${cdir}/hardware.yaml"
fi
docker cp -q "${profile_path}" "${container}:${cdir}/profile.yaml"

# --- stack ------------------------------------------------------------------------------

log "stack: domain ${domain}, endpoint :${port}, profile ${profile_name}"
smear_names=()
endpoint_remap=""
if ((lidar_smear)); then   # rig.yaml lidar_smear: true; crane's cloud goes to /points/instant, smeared onto /points
    endpoint_remap=" -r /points:=/points/instant"
    smear_names=(lidar_smear)
fi
cstart endpoint "ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:=${port}${endpoint_remap}"
((lidar_smear)) && { cstart lidar_smear "ros2 run mhseals_nav lidar_smear"; stack_check+=(lidar_smear); }
if ((stack_resolve)); then   # a current profile: mhseals_testing resolves its device parts (stack.py)
    docker cp -q "${here}/stack.py" "${container}:${cdir}/stack.py"
    while IFS=$'\t' read -r name argv; do
        stack_names+=("${name}"); stack_cmds+=("${argv}"); stack_check+=("${name}")
    done < <(cexec "${ros_env} cd ${cdir}; python3 -P stack.py profile.yaml")
    ((${#stack_names[@]})) || { log "profile ${profile_name} starts no stack component (resolved with ${install})"; exit 2; }
fi
for i in "${!stack_names[@]}"; do cstart "${stack_names[i]}" "${stack_cmds[i]}"; done
# The driver chain boat_test arms (mhseals_testing.ros.arming.driver_commands), on the topic backend.
driver_names=()
if ((driver)); then
    while IFS=$'\t' read -r name argv; do
        cstart "driver_${name}" "${argv}"; driver_names+=("driver_${name}"); stack_check+=("driver_${name}")
    done < <(cexec "${ros_env} cd ${cdir}; python3 -P -c '
import shlex, sys
from mhseals_testing.profiles import load_profile
from mhseals_testing.ros.arming import driver_commands
from dataclasses import replace
profile = load_profile(\"profile.yaml\")
profile = replace(profile, outputs=\"topic\")
if sys.argv[1] == \"velocity\":
    profile = profile.with_velocity_control()
for name, argv in driver_commands(profile, \"${cdir}/hardware.yaml\"):
    print(name, shlex.join(argv), sep=\"\t\")
' ${mode}")
    ((${#driver_names[@]})) || { log "could not resolve the driver chain"; exit 1; }
fi
# A warm ros2 daemon lets --run commands resolve topic types at once.
cexec "${ros_env} ros2 daemon start >/dev/null 2>&1" || true
# progress.json for rigq's watchdog: sim time, message counts, commanded motion and travel.
((probe)) && cstart probe "python3 probe.py ${cdir}/progress.json $(printf '%q' "${probe_config}")"
cstart recorder "ros2 bag record -o ${cdir}/bag --include-unpublished-topics --include-hidden-topics --topics ${record[*]}"

# --- player -------------------------------------------------------------------------------

player_cmd=(nice -n "${nice}" "${player}" -batchmode -screen-fullscreen 0 -screen-width 640 -screen-height 360
    -logFile "${session}/player.log"
    --crane-profile "${runtime_profile}" --crane-scene "${scene}"
    --crane-ros-ip 127.0.0.1 --crane-ros-port "${port}"
    "${player_args[@]}"
    --crane-roboboat-course "${course_path}"
    --crane-roboboat-course-truth "${session}/course_truth.json")
[[ -n "${faults}" ]] && player_cmd+=(--crane-calibration-faults "$(cd "$(dirname "${faults}")" && pwd)/$(basename "${faults}")")
if [[ "${boat_profile}" == *.json ]]; then
    player_cmd+=(--crane-boat-profile-json "$(cd "$(dirname "${boat_profile}")" && pwd)/$(basename "${boat_profile}")")
elif [[ -n "${boat_profile}" ]]; then
    player_cmd+=(--crane-boat-profile "${boat_profile}")
fi
[[ -n "${robot}" ]] && player_cmd+=(--crane-robot "${robot}")
[[ -n "${current}" ]] && player_cmd+=(--crane-water-current "${current}")
[[ -n "${wind}" ]] && player_cmd+=(--crane-wind "${wind}")
[[ -n "${gps_sigma}" ]] && player_cmd+=(--crane-gps-sigma "${gps_sigma}")
player_cmd+=(--crane-seed "${seed}" --crane-contract-seed "${seed}" --crane-sim-profile "${sim_profile}")
player_cmd+=("${extra[@]}")

log "player: ${course_path##*/}, seed ${seed}${faults:+, faults ${faults##*/}}${current:+, current ${current}}${wind:+, wind ${wind}}"
DISPLAY="${display}" SDL_VIDEODRIVER=x11 setsid "${player_cmd[@]}" >/dev/null 2>&1 &
player_pid=$!
write_run_env

deadline=$(($(date +%s) + ready_timeout))
until grep -q CRANE_ROBOBOAT_COURSE_READY "${session}/player.log" 2>/dev/null; do
    kill -0 "${player_pid}" 2>/dev/null || { log "player exited before the course was ready"; exit 1; }
    (($(date +%s) < deadline)) || { log "no CRANE_ROBOBOAT_COURSE_READY in ${ready_timeout}s"; exit 1; }
    sleep 0.5
done
grep -q CRANE_ROBOBOAT_COURSE_FAILED "${session}/player.log" && { log "course failed"; exit 1; }
ready=$(now_ms)
# A stack that died at start-up (bad domain, missing package) must not look ready.
for name in "${stack_check[@]}"; do
    cexec "kill -0 \$(cat ${cdir}/${name}.pid 2>/dev/null) 2>/dev/null" ||
        { log "${name} is not running; see ${name}.log"; exit 1; }
done
log "course ready"
# Results are only attributable with the plant that produced them (profile, sha256, status).
plant_line="$(grep -m1 -o 'CRANE_PLANT_APPLIED .*' "${session}/player.log" || true)"
log "plant: ${plant_line:-none (no CRANE_PLANT_APPLIED: legacy physics?)}"
docker cp -q "${session}/course_truth.json" "${container}:${cdir}/course_truth.json"

# --- drive or run, then hold until the duration ends ----------------------------------------

if [[ -n "${run_cmd}" ]]; then
    log "run: ${run_cmd}"
    # CMD goes through bash -c (env prefixes, pipes); its status lands in run.status.
    run_inner="timeout -s INT ${duration} bash -c $(printf '%q' "${run_cmd}"); echo \$? >${cdir}/run.status"
    cstart run "bash -c $(printf '%q' "${run_inner}")"
elif ((drive)); then
    cstart drive "python3 drive.py ${drive_topic} '${drive_schedule}'"
fi
hold_rc=0
remaining=$((duration * 1000 - ($(now_ms) - ready)))
# Hold in the background so that INT/TERM run the cleanup trap at once.
if ((remaining > 0)); then
    # A --run that exits ends the recording early (its status file appears).
    hold_until=$(( $(now_ms) + remaining ))
    ( while (( $(now_ms) < hold_until )); do
          kill -0 "${player_pid}" 2>/dev/null || exit 3
          [[ -n "${run_cmd}" ]] && docker exec "${container}" test -e "${cdir}/run.status" && exit 0
          sleep 1
      done ) &
    hold_pid=$!
    wait "${hold_pid}" || hold_rc=$?
    hold_pid=""
fi

cstop INT run drive
cstop INT recorder probe
recorded_s=$(( ($(now_ms) - ready) / 1000 ))
# The player must outlive the scenario and the stop steps: a crash after ready fails the run, whatever its status.
if ((hold_rc == 3)) || ! kill -0 "${player_pid}" 2>/dev/null; then
    log "player exited before the scenario finished"; exit 1
fi
log "recorded ${recorded_s}s"
outcome=null
if [[ -n "${task}" ]]; then
    outcome="$(cexec "${ros_env} cd ${cdir}; python3 -P post.py ${cdir} ${task}" | tail -1)" || outcome=null
    log "task ${task}: ${outcome}"
fi
# Provenance from the container: the effective nav2_params and each package's synced commit.
provenance="$(cexec "${ros_env} for p in mhseals_hardware mhseals_nav mhseals_testing; do
        prefix=\$(ros2 pkg prefix \$p 2>/dev/null) || { echo \"  \$p: null\"; continue; }
        src=\$(readlink -f \$prefix/../../src/\$p 2>/dev/null)
        echo \"  \$p: {prefix: \$prefix, commit: '\$(cat \$src/.source_commit 2>/dev/null)'}\"
    done
    echo \"nav2_params_sha256: \$(sha256sum \$(ros2 pkg prefix mhseals_nav)/share/mhseals_nav/config/nav2_params.yaml | cut -c1-64)\"
    echo \"hardware_sha256: \$(sha256sum ${cdir}/hardware.yaml | cut -c1-64)\"")"


# 124 is timeout cutting the run off at the duration; anything else non-zero is a failure.
run_status="$(cexec "cat ${cdir}/run.status 2>/dev/null" || true)"
run_failed=0
if [[ -n "${run_cmd}" && "${run_status:-0}" != 0 && "${run_status}" != 124 ]]; then
    run_failed=1
    log "run command failed (exit ${run_status})"
fi

cat >"${session}/session.yaml" <<EOF
profile: {name: ${profile_name}, sim: true, ground_truth: /crane/odom, path: ${profile_path}}
clock: {domain: sim, time_scale: 1.0}
course: ${course_path}
faults: ${faults:-null}
faults_sha256: $([[ -n "${faults}" ]] && sha256sum "${faults}" | cut -c1-64 || echo null)
current: ${current:-null}
wind: ${wind:-null}
gps_sigma: ${gps_sigma:-null}
seed: ${seed}
mode: ${mode}
hardware: ${hardware}
overlay: ${overlay:-null}
install: ${install}
packages:
${provenance}
player_argv: $(python3 -I -c 'import json,sys;print(json.dumps(sys.argv[1:]))' "${player_cmd[@]}")
run: $(python3 -I -c 'import json,sys;print(json.dumps(sys.argv[1]))' "${run_cmd}")
run_status: ${run_status:-null}
task_outcome: ${outcome:-null}
duration_s: ${duration}
recorded_s: ${recorded_s}
ros: {domain: ${domain}, endpoint_port: ${port}}
player: {path: ${player}, runtime_profile: ${runtime_profile}, build: $(python3 -I -c "import json,sys,yaml;m=json.load(open(sys.argv[1]));print(json.dumps({'commit':m['sourceCommit'] or sys.argv[2],'guid':m['buildGuid']}))" "$(dirname "${player}")/crane-build-manifest.json" "${player_commit}")}
ready_after_s: $(((ready - started) / 1000))
EOF

if ((smoke)); then
    log "smoke gates"
    cexec "${ros_env} cd ${cdir}; python3 check.py ${cdir} ${cdir}/rig.yaml ${duration}"
fi

((run_failed == 0)) || exit 1
