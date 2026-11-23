#!/usr/bin/env bash
# Checks package-firmware.sh with host-built ELF files and the host objcopy.
set -euo pipefail

script="$(cd "$(dirname "$0")" && pwd)/package-firmware.sh"
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
cd "$work"
export OBJCOPY=${OBJCOPY:-objcopy}
failures=0

check() {
    local description=$1 expected=$2
    shift 2
    local status=0
    : > outputs
    GITHUB_OUTPUT="$work/outputs" "$@" > log 2>&1 || status=$?
    if [ "$expected" = ok ] && [ "$status" -ne 0 ]; then
        echo "FAIL: ${description} (exit ${status})"; cat log; failures=$((failures + 1))
    elif [ "$expected" = fail ] && [ "$status" -eq 0 ]; then
        echo "FAIL: ${description} (expected a failure)"; cat log; failures=$((failures + 1))
    else
        echo "ok: ${description}"
    fi
}

elf() {
    mkdir -p "$(dirname "$1")"
    echo 'int main() { return 0; }' > main.c
    cc -o "$1" main.c
}

mkdir -p build/BOARD/lib/RelWithDebInfo
check "no application and no ELF publishes nothing" ok "$script" build BOARD RelWithDebInfo v1.0.0 ""
grep -qx "packaged=false" outputs || { echo "FAIL: packaged=false not written"; failures=$((failures + 1)); }
[ ! -e e_robot_arm-firmware-BOARD-v1.0.0.tar.gz ] || { echo "FAIL: an archive was made"; failures=$((failures + 1)); }

check "a declared application that was not built fails" fail "$script" build BOARD RelWithDebInfo v1.0.0 "arm"

elf build/BOARD/targets/arm/RelWithDebInfo/arm.elf
check "an ELF nobody declared fails" fail "$script" build BOARD RelWithDebInfo v1.0.0 ""

check "a declared application is packaged with its hex" ok "$script" build BOARD RelWithDebInfo v1.0.0 "arm"
grep -qx "packaged=true" outputs || { echo "FAIL: packaged=true not written"; failures=$((failures + 1)); }
tar -tzf e_robot_arm-firmware-BOARD-v1.0.0.tar.gz | sort > contents
printf 'BOARD/\nBOARD/arm.elf\nBOARD/arm.hex\n' | diff - contents || { echo "FAIL: archive contents"; failures=$((failures + 1)); }

elf build/BOARD/targets/boot/RelWithDebInfo/boot.elf
check "two declared applications are both packaged" ok "$script" build BOARD RelWithDebInfo v1.0.1 "boot arm"
tar -tzf e_robot_arm-firmware-BOARD-v1.0.1.tar.gz | sort > contents
printf 'BOARD/\nBOARD/arm.elf\nBOARD/arm.hex\nBOARD/boot.elf\nBOARD/boot.hex\n' | diff - contents || { echo "FAIL: archive contents with two applications"; failures=$((failures + 1)); }
check "one of two ELFs not declared fails" fail "$script" build BOARD RelWithDebInfo v1.0.1 "boot"
rm build/BOARD/targets/boot/RelWithDebInfo/boot.elf

elf build/BOARD/targets/arm/Debug/arm.elf
check "an ELF of another configuration is ignored" ok "$script" build BOARD RelWithDebInfo v1.0.0 "arm"

elf build/BOARD/other/RelWithDebInfo/arm.elf
check "two ELFs of one application fail" fail "$script" build BOARD RelWithDebInfo v1.0.0 "arm"

if [ "$failures" -ne 0 ]; then
    echo "${failures} check(s) failed"
    exit 1
fi
echo "all checks passed"
