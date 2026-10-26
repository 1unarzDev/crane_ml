#!/usr/bin/env bash
# The rig's own ROS container: private IPC (the host /dev/shm Fast DDS state wedges a
# --ipc host container), --init (tini reaps the process groups runs leave behind), host network
# (the player on the host reaches the ROS-TCP endpoint on 127.0.0.1).
#
# Usage: container.sh up|down|status [--config PATH]
#   rig.yaml container, container_image and container_mounts (host:container[:ro]) set it up;
#   the container carries the label crane.rig=1 and down only removes a container with that label.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cmd=${1:-status}; shift || true
config="${here}/rig.yaml"
[[ "${1:-}" == --config ]] && config=$2
eval "$(python3 -I - "${here}" "${config}" <<'EOF'
import shlex, sys
sys.path.insert(0, sys.argv[1])
import rigconf
rig = rigconf.load(sys.argv[2])
print(f"name={shlex.quote(rig['container'])} image={shlex.quote(rig.get('container_image') or '')}")
print('mounts=(' + ' '.join(shlex.quote(m) for m in rig.get('container_mounts') or []) + ')')
EOF
)"
ours() { [[ "$(docker inspect -f '{{index .Config.Labels "crane.rig"}}' "${name}" 2>/dev/null)" == 1 ]]; }
case "${cmd}" in
    status)
        docker inspect -f '{"name": "{{.Name}}", "running": {{.State.Running}}, "ipc": "{{.HostConfig.IpcMode}}", "init": {{.HostConfig.Init}}}' "${name}" 2>/dev/null ||
            echo "{\"name\": \"${name}\", \"running\": false}" ;;
    up)
        if docker inspect "${name}" >/dev/null 2>&1; then
            ours || { echo "container ${name} exists and is not the rig's (no crane.rig label)" >&2; exit 2; }
            docker start "${name}" >/dev/null
        else
            [[ -n "${image}" ]] || { echo "set container_image in rig.local.yaml" >&2; exit 2; }
            args=()
            for m in "${mounts[@]}"; do args+=(-v "${m}"); done
            docker run -d --name "${name}" --label crane.rig=1 --init --ipc private --network host \
                --user roboboat -e BASH_ENV= "${args[@]}" --entrypoint sleep "${image}" infinity >/dev/null
        fi
        echo "${name} up" ;;
    down)
        ours || { echo "refusing: ${name} is not the rig's container" >&2; exit 2; }
        docker rm -f "${name}" >/dev/null && echo "${name} removed" ;;
    *) sed -n '2,/^set/s/^# \{0,1\}//p' "$0" | head -n -1; exit 2 ;;
esac
