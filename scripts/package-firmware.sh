#!/usr/bin/env bash
# Packages a board's firmware for a release, or says why there is none.
#
# usage: package-firmware.sh <build-dir> <target> <configuration> <version> "<applications>"
#
# <applications> lists the firmware applications the release promises for <target>, by the ELF's base name,
# separated by spaces. Each must have exactly one <name>.elf under <build-dir>/<target>/**/<configuration>/;
# its <name>.hex is made with ${OBJCOPY:-arm-none-eabi-objcopy}, both go into
# e_robot_arm-firmware-<target>-<version>.tar.gz, and the archive is checked before it is kept.
# With no applications listed, nothing is packaged, and an ELF found anyway fails the run so a new
# application cannot be left out of a release unnoticed.
# Writes packaged=true|false to $GITHUB_OUTPUT when it is set; exits non-zero on any failure.
set -euo pipefail

build_dir=$1
target=$2
configuration=$3
version=$4
read -r -a applications <<< "${5:-}"
objcopy=${OBJCOPY:-arm-none-eabi-objcopy}
archive="e_robot_arm-firmware-${target}-${version}.tar.gz"

output() {
    if [ -n "${GITHUB_OUTPUT:-}" ]; then
        echo "$1" >> "$GITHUB_OUTPUT"
    fi
}

summary() {
    echo "$1"
    if [ -n "${GITHUB_STEP_SUMMARY:-}" ]; then
        echo "$1" >> "$GITHUB_STEP_SUMMARY"
    fi
}

fail() {
    echo "::error::${target}: $1" >&2
    exit 1
}

mapfile -t elves < <(find "${build_dir}/${target}" -name "*.elf" -path "*/${configuration}/*" -type f 2> /dev/null | sort)

for elf in "${elves[@]}"; do
    name=$(basename "$elf" .elf)
    declared=false
    for application in "${applications[@]}"; do
        if [ "$application" = "$name" ]; then
            declared=true
        fi
    done
    [ "$declared" = true ] || fail "firmware ${name}.elf was built but is not listed in the release's applications"
done

if [ "${#applications[@]}" -eq 0 ]; then
    summary "${target}: no firmware application is defined; nothing to publish."
    output "packaged=false"
    exit 0
fi

staging=$(mktemp -d)
trap 'rm -rf "$staging"' EXIT
mkdir -p "${staging}/${target}"

for application in "${applications[@]}"; do
    matches=()
    for elf in "${elves[@]}"; do
        if [ "$(basename "$elf" .elf)" = "$application" ]; then
            matches+=("$elf")
        fi
    done
    [ "${#matches[@]}" -eq 1 ] || fail "expected one ${application}.elf for ${configuration}, found ${#matches[@]}"
    cp "${matches[0]}" "${staging}/${target}/${application}.elf"
    "$objcopy" -O ihex "${matches[0]}" "${staging}/${target}/${application}.hex"
    [ -s "${staging}/${target}/${application}.hex" ] || fail "${application}.hex is empty"
done

tar -czf "$archive" -C "$staging" "$target"

contents=$(tar -tzf "$archive")
for application in "${applications[@]}"; do
    for file in "${application}.elf" "${application}.hex"; do
        grep -qx "${target}/${file}" <<< "$contents" || fail "${archive} lacks ${file}"
    done
done

summary "${target}: packaged ${archive} with ${applications[*]}"
output "packaged=true"
