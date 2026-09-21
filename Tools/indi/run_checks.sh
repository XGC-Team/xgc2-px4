#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
build=${1:-"$root/build/indi-host"}
cmake -S "$root/Tools/indi" -B "$build" -DINDI_SANITIZERS=ON
cmake --build "$build" --parallel 2
# Separate concurrent adversarial tracks. These are automated tests, not independent reviewers.
"$build/indi_core_test" > "$build/core-results.txt" 2>&1 & p1=$!
OPENBLAS_NUM_THREADS=1 python3 "$root/Tools/indi/check_math.py" > "$build/math-results.txt" 2>&1 & p2=$!
python3 "$root/Tools/indi/check_contracts.py" > "$build/contract-results.txt" 2>&1 & p3=$!
s=0
wait "$p1" || s=1
wait "$p2" || s=1
wait "$p3" || s=1
cat "$build/core-results.txt" "$build/math-results.txt" "$build/contract-results.txt"
exit "$s"
