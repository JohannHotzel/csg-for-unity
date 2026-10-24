# CSG for Unity

Konvexe Körper von geschlossenen Netzen abziehen. Das Ergebnis ist nach jedem Schnitt geschlossen und manifold,
und die Schnittflächen bekommen ihr eigenes Material (Submesh 1), damit Körper ein Innenleben zeigen.

Mehr kann die Bibliothek bewusst nicht: keine Vereinigung, kein Schnitt mit nicht-konvexen Körpern, kein Zerbrechen.

## Installation

Benötigt Unity 6 (6000.0 oder neuer). Die Bibliothek hängt von keinem anderen Paket ab. Die Demo braucht zusätzlich
die Universal Render Pipeline und das Input-System-Paket.

### Von GitHub

Git muss installiert und im `PATH` sein, denn Unity lädt das Paket damit herunter.

1. **Window → Package Manager** öffnen.
2. **+** → **Install package from git URL…** wählen.
3. Die URL eingeben und **Install** klicken:

   ```
   https://github.com/JohannHotzel/csg-for-unity.git
   ```

Für eine feste Version hängt man ihr Tag an, z. B. `https://github.com/JohannHotzel/csg-for-unity.git#v0.1.0`.
Ohne Tag bekommt man den neuesten Stand von `main`. Unity merkt sich den genauen Commit in
`Packages/packages-lock.json`; zum Aktualisieren die URL einfach erneut installieren.

Alternativ trägt man die Abhängigkeit selbst in `Packages/manifest.json` ein:

```json
{
  "dependencies": {
    "com.johann.csg-for-unity": "https://github.com/JohannHotzel/csg-for-unity.git#v0.1.0"
  }
}
```

### Aus einer lokalen Kopie

Das Repository klonen, dann im Package Manager **+** → **Install package from disk…** wählen und seine
`package.json` auswählen. Das Paket bleibt so bearbeitbar; das ist der Weg, wenn man an der Bibliothek selbst
arbeiten will.

### Demo

Im Package Manager **CSG for Unity** auswählen, den Reiter **Samples** öffnen und **Demo** importieren. Unity kopiert
sie nach `Assets/Samples/CSG for Unity/<Version>/Demo`. `CsgDemo.unity` öffnen und Play drücken:

| Eingabe | Wirkung |
|---|---|
| Linke Maustaste | gewählte Form aus dem Körper unter dem Mauszeiger schneiden |
| 1 – 4 | Kugel, Quader, Zylinder, Brocken (jede Form hat eigene Einstellungen am Objekt `Demo`) |
| Rechte Maustaste / mittlere Maustaste / Mausrad | drehen / verschieben / zoomen |

## Schnellstart

1. Ein Objekt mit `MeshFilter` + `MeshRenderer`, dessen Mesh geschlossen ist (Unity-Primitives, die meisten Modelle;
   bei importierten Modellen **Read/Write** aktivieren).
2. `CsgBody` hinzufügen, `Inner Material` setzen. Ein vorhandener `MeshCollider` wird mitgeführt.
3. Schneiden (Weltkoordinaten):

```csharp
using CsgForUnity;

var r = body.Subtract(CsgCutters.Sphere(hit.point, 0.4f, Random.rotation));
if (!r.Ok) Debug.Log(r.Reason);          // Netz ist dann unverändert

CsgBody.SubtractAll(CsgCutters.Box(center, halfExtents, rotation));   // alle aktiven Körper, die er berührt
```

Schneidkörper: `CsgCutters.Sphere`, `Box`, `Cylinder`, `Rock`, `FromPoints` (konvexe Hülle), `FromMesh`.
`CsgCutters.ToMesh` liefert ein Vorschau-Mesh. `body.ResetToSource()` verwirft alle Schnitte.

## Verhalten

- Jeder Schnitt wird vollständig geprüft, bevor er übernommen wird (keine offenen, doppelten, entarteten oder
  umgeklappten Dreiecke). Scheitert ein Versuch an einem numerischen Grenzfall, wird mit einem minimal veränderten
  Körper wiederholt (zuerst um 0,2 ‰ der Netzdiagonale vergrößert, im letzten Versuch bis 3 ‰ verschoben/gedreht).
  Scheitert auch das, bleibt das Netz unverändert und `CutResult.Ok` ist `false` – im Stresstest bei weniger als
  0,1 % der Schnitte.
- Außenflächen behalten Normalen und UVs des Quellnetzes, Schnittflächen bekommen planare UVs im Weltmaßstab
  (`innerUvScale`), auch bei ungleichmäßig skalierten Objekten unverzerrt.
- Nach dem Schnitt werden kurze Kanten und Nadeldreiecke lokal aufgeräumt (ändert die Form höchstens um
  2·10⁻⁵ der Diagonale).
- Alle Submeshes des Quellnetzes werden zu Submesh 0 (ein Außenmaterial).

## Aufbau

| Teil | Inhalt |
|---|---|
| `Runtime/Core` | reines C# ohne Unity-Abhängigkeit: `CsgMesh` (Netz in double), `ConvexCutter` (Körper, Hülle), `Csg.Subtract`, `Triangulator`, `CsgCleanup`, `CsgAnalyze`, `CsgShapes` |
| `Runtime` | `CsgBody` (Komponente, Unity-Mesh mit zwei Submeshes), `CsgCutters` (Körper mit Unity-Typen) |
| `Tests/Editor` | EditMode-Tests (Test Runner) |
| `Tests~/DotNet` | Kern-Tests außerhalb von Unity: `./run-tests.sh [N] [Seeds…]`, Volumen-Gegenprobe mit Manifold |
| `Samples~/Demo` | Demo-Szene, im Package Manager unter „Samples“ importierbar |

## Leistung (Apple Silicon, Unity-Editor, Release)

Kugel-Schnitt (r = 0,15) in einer Kugel mit … Dreiecken, Median:

| Dreiecke | 5 k | 20 k | 82 k | 328 k |
|---|---|---|---|---|
| Schnitt | 1,7 ms | 2,3 ms | 4,3 ms | 10,5 ms |

Dazu kommt der Neuaufbau des Unity-Meshes, der linear mit der Netzgröße wächst (82 k Dreiecke: ~15 ms plus
~12 ms MeshCollider-Cooking). Für sehr große Netze wäre eine Aufteilung in Kacheln der nächste Schritt.
