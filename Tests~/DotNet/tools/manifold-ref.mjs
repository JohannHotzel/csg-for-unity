// Gegenprobe: dieselben Schnittfolgen (tatsächlich verwendete Körper aus dem C#-Stresstest) mit Manifold nachrechnen
// und das Endvolumen vergleichen. Aufruf über run-tests.sh (JavaScriptCore, globalThis.MFDIR / JSON_FILE).
globalThis.console ??= { log: print, error: print, warn: print };
globalThis.URL ??= class { constructor(u) { this.href = String(u); } toString() { return this.href; } };

const mod = await import(globalThis.MFDIR + '/manifold.js');
const W = await mod.default({ wasmBinary: readFile(globalThis.MFDIR + '/manifold.wasm', 'binary'), print, printErr: print });
W.setup();

const data = JSON.parse(readFile(globalThis.JSON_FILE));
let fails = 0;
for (const sc of data) {
  // nach Position verschweißen (Manifold braucht geschlossene Topologie)
  const key = new Map(), vp = [], map = [], P = sc.positions;
  for (let i = 0; i < P.length / 3; i++) {
    const k = P[3 * i] + ',' + P[3 * i + 1] + ',' + P[3 * i + 2];
    let v = key.get(k);
    if (v === undefined) { v = vp.length / 3; key.set(k, v); vp.push(P[3 * i], P[3 * i + 1], P[3 * i + 2]); }
    map.push(v);
  }
  const mesh = new W.Mesh({ numProp: 3, vertProperties: new Float32Array(vp), triVerts: new Uint32Array(sc.indices.map(i => map[i])) });
  mesh.merge();
  let ref = new W.Manifold(mesh);
  for (const pts of sc.cutters) {
    const h = W.Manifold.hull(pts), n = ref.subtract(h);
    h.delete(); ref.delete(); ref = n;
  }
  const rv = ref.volume();
  ref.delete();
  const rel = Math.abs(sc.volume - rv) / Math.max(1e-9, Math.abs(rv));
  const ok = rel < 1e-4;
  if (!ok) fails++;
  print(`${ok ? '  ok    ' : '  FEHLER'} ${sc.name}: V ${sc.volume.toFixed(6)} vs Manifold ${rv.toFixed(6)} (rel ${rel.toExponential(1)}), ${sc.cutters.length} Schnitte`);
}
print(fails ? `\n${fails} Abweichungen` : '\nalle Volumina stimmen mit Manifold überein');
if (fails) throw new Error('Volumen weicht ab');
