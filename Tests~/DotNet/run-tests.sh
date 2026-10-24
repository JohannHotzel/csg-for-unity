#!/bin/sh
# Tests of the pure C# core outside Unity (macOS):
#   1. smoke  – single cases with exactly known volume, cut-face material, normals
#   2. stress – long cut sequences, closed/manifold after every cut, no self-intersection at the end
#   3. cross-check – the same cutters replayed with Manifold (WASM in JavaScriptCore), final volume must match
# Usage: ./run-tests.sh [N] [seeds…]     (default: N=300, seed 0)    bench: ./run-tests.sh bench
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
WORK=${TMPDIR:-/tmp}/csg-for-unity-test
JSC=/System/Library/Frameworks/JavaScriptCore.framework/Versions/Current/Helpers/jsc
mkdir -p "$WORK"
dotnet build -c Release "$HERE/CsgTests.csproj" -v q -nologo | grep -E "error|Warn|warn" || true
BIN="$HERE/bin/Release/net10.0/CsgTests.dll"
if [ "$1" = bench ]; then dotnet "$BIN" bench; exit; fi
N=${1:-300}; shift || true
SEEDS=${*:-0}
dotnet "$BIN" smoke
for f in manifold.js manifold.wasm; do
  [ -f "$WORK/$f" ] || curl -sL "https://cdn.jsdelivr.net/npm/manifold-3d@3.5.4/$f" -o "$WORK/$f"
done
for s in $SEEDS; do
  dotnet "$BIN" stress "$N" "$s" "$WORK/stress-$s.json"
  echo "== Manifold cross-check, seed $s"
  printf 'globalThis.MFDIR = "%s";\nglobalThis.JSON_FILE = "%s";\nawait import("%s/tools/manifold-ref.mjs");\n' "$WORK" "$WORK/stress-$s.json" "$HERE" > "$WORK/ref-$s.mjs"
  "$JSC" -m "$WORK/ref-$s.mjs"
done
