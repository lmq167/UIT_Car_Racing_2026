#!/usr/bin/env bash
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
mkdir -p "$root/build"
cd "$root/build"
cmake .. "$@"
cmake --build . -- -j2
