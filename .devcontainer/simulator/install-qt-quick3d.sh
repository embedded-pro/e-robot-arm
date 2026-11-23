#!/usr/bin/env bash
# Installs Qt Quick 3D, Mesa's software renderer and a virtual display for the simulator.
# Used by the simulator Dev Container and the simulator CI job.
set -euo pipefail

SUDO=()
if [ "$(id -u)" -ne 0 ] && command -v sudo > /dev/null; then SUDO=(sudo); fi

# The image's Cloudsmith source fails apt-get update; the simulator does not need it.
mapfile -t stale < <(grep -rl 'dl.cloudsmith.io' /etc/apt/sources.list /etc/apt/sources.list.d 2> /dev/null || true)
if [ "${#stale[@]}" -ne 0 ]; then "${SUDO[@]}" rm -f "${stale[@]}"; fi
"${SUDO[@]}" apt-get update

# Debian and Ubuntu releases name the Qt Quick 3D QML modules differently.
available() {
    for name in "$@"; do
        if apt-cache policy "$name" | grep -q 'Candidate: [0-9]'; then
            echo "$name"
            return
        fi
    done
}

"${SUDO[@]}" apt-get install -y --no-install-recommends \
    qt6-declarative-dev qt6-declarative-dev-tools qt6-quick3d-dev qt6-shadertools-dev \
    qml6-module-qtqml qml6-module-qtqml-workerscript qml6-module-qtquick qml6-module-qtquick-window \
    "$(available qml6-module-qtquick3d qml6-module-quick3d)" \
    "$(available qml6-module-qtquick3d-helpers qml6-module-quick3d-helpers)" \
    libgl1-mesa-dri xvfb xauth
