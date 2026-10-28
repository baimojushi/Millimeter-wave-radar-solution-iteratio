#!/usr/bin/env bash
set -eu
cd "$(dirname "$0")"
CXX="${CXX:-g++}"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT
FLAGS="-std=c++11 -O2 -Wall -Wextra -Werror"
echo "== v8 core adversarial tests =="
"$CXX" $FLAGS test_v8.cpp -o "$OUT/core"
"$OUT/core"
echo "== v8 edge tests =="
"$CXX" $FLAGS test_edge_v8.cpp -o "$OUT/edge"
"$OUT/edge"
echo "== v8 ESP32-interface stub + protocol tests =="
"$CXX" $FLAGS -Itest_stubs -I. test_protocol_v8.cpp -o "$OUT/protocol"
"$OUT/protocol"
echo "== v8 stress/known-limitations (diagnostic, NOT a pass/fail suite) =="
"$CXX" $FLAGS test_stress_v8.cpp -o "$OUT/stress"
"$OUT/stress"
echo "== v7 ACF fragment reproduced (NOT full v7 firmware) =="
"$CXX" $FLAGS repro_v7_halving.cpp -o "$OUT/v7"
"$OUT/v7"
if [ "${RUN_UBSAN:-0}" = "1" ]; then
    echo "== undefined behavior sanitizer =="
    for f in test_v8.cpp test_edge_v8.cpp test_protocol_v8.cpp test_stress_v8.cpp; do
        "$CXX" -std=c++11 -O1 -g -fsanitize=undefined \
          -fno-sanitize-recover=all -Itest_stubs -I. "$f" -o "$OUT/ubsan"
        "$OUT/ubsan" >/dev/null
        echo "UBSan PASS: $f"
    done
fi
