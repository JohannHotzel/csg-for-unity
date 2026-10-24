#!/bin/sh
# Tests des reinen C#-Kerns außerhalb von Unity (macOS):
#   1. smoke  – Einzelfälle mit exakt bekanntem Volumen, Material der Schnittflächen, Normalen
#   2. stress – lange Schnittfolgen, nach jedem Schnitt geschlossen/manifold, am Ende keine Selbstdurchdringung
#   3. Gegenprobe – dieselben Körper mit Manifold (WASM in JavaScriptCore), Endvolumen muss übereinstimmen
# Aufruf: ./run-tests.sh [N] [Seeds…]     (Standard: N=300, Seed 0)    bench: ./run-tests.sh bench
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
  echo "== Manifold-Gegenprobe Seed $s"
  printf 'globalThis.MFDIR = "%s";\nglobalThis.JSON_FILE = "%s";\nawait import("%s/tools/manifold-ref.mjs");\n' "$WORK" "$WORK/stress-$s.json" "$HERE" > "$WORK/ref-$s.mjs"
  "$JSC" -m "$WORK/ref-$s.mjs"
done
