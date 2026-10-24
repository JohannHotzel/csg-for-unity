# CSG for Unity

Subtract convex bodies from closed meshes. After every cut the result is closed and manifold, and the cut faces
get their own material (submesh 1), so objects show an interior.

The library deliberately does nothing more: no union, no cutting with non-convex bodies, no fracturing.

## Installation

Requires Unity 6 (6000.0 or newer). The library has no package dependencies. The demo sample additionally needs
the Universal Render Pipeline and the Input System package.

### From GitHub

Git must be installed and on the `PATH`, because Unity fetches the package with it.

1. Open **Window → Package Manager**.
2. Click **+** → **Install package from git URL…**
3. Enter the URL and click **Install**:

   ```
   https://github.com/JohannHotzel/csg-for-unity.git
   ```

To pin a released version, append its tag, e.g. `https://github.com/JohannHotzel/csg-for-unity.git#v0.1.0`.
Without a tag you get the latest state of `main`. Unity records the exact commit in `Packages/packages-lock.json`;
to update later, install the URL again.

Alternatively, add the dependency to `Packages/manifest.json` yourself:

```json
{
  "dependencies": {
    "com.johann.csg-for-unity": "https://github.com/JohannHotzel/csg-for-unity.git#v0.1.0"
  }
}
```

### From a local copy

Clone the repository, then in the Package Manager click **+** → **Install package from disk…** and select its
`package.json`. The package stays editable, which is the way to go if you want to work on the library itself.

### Demo

In the Package Manager, select **CSG for Unity**, open the **Samples** tab and import **Demo**. Unity copies it to
`Assets/Samples/CSG for Unity/<version>/Demo`. Open `CsgDemo.unity` and press Play:

| Input | Action |
|---|---|
| Left click | cut the selected shape out of the body under the cursor |
| 1 – 4 | sphere, box, cylinder, rock (each shape has its own settings on the `Demo` object) |
| Right mouse / middle mouse / wheel | orbit / pan / zoom |

## Quick start

1. An object with `MeshFilter` + `MeshRenderer` whose mesh is closed (Unity primitives, most models;
   enable **Read/Write** for imported models).
2. Add `CsgBody` and set `Inner Material`. An existing `MeshCollider` is kept up to date.
3. Cut (world space):

```csharp
using CsgForUnity;

var r = body.Subtract(CsgCutters.Sphere(hit.point, 0.4f, Random.rotation));
if (!r.Ok) Debug.Log(r.Reason);          // the mesh is unchanged in that case

CsgBody.SubtractAll(CsgCutters.Box(center, halfExtents, rotation));   // every active body it touches
```

Cutters: `CsgCutters.Sphere`, `Box`, `Cylinder`, `Rock`, `FromPoints` (convex hull), `FromMesh`.
`CsgCutters.ToMesh` returns a preview mesh. `body.ResetToSource()` discards all cuts.

## Behavior

- Every cut is fully validated before it is committed (no open, duplicate, degenerate or flipped triangles).
  If an attempt fails on a numerical edge case, it is retried with a minimally modified cutter (first enlarged by
  0.2 ‰ of the mesh diagonal, in the last attempt moved/rotated by up to 3 ‰). If that fails too, the mesh stays
  unchanged and `CutResult.Ok` is `false` – in the stress test for below 0.1 % of the cuts.
- Outer faces keep the normals and UVs of the source mesh; cut faces get planar UVs at world scale
  (`innerUvScale`), undistorted even on non-uniformly scaled objects.
- After the cut, short edges and needle triangles are cleaned up locally (changes the shape by at most
  2·10⁻⁵ of the diagonal).
- All submeshes of the source mesh become submesh 0 (one outer material).

## Structure

| Part | Contents |
|---|---|
| `Runtime/Core` | pure C# without Unity dependencies: `CsgMesh` (mesh in double precision), `ConvexCutter` (cutter, hull), `Csg.Subtract`, `Triangulator`, `CsgCleanup`, `CsgAnalyze`, `CsgShapes` |
| `Runtime` | `CsgBody` (component, Unity mesh with two submeshes), `CsgCutters` (cutters from Unity types) |
| `Tests/Editor` | EditMode tests (Test Runner) |
| `Tests~/DotNet` | core tests outside Unity: `./run-tests.sh [N] [seeds…]`, volume cross-check against Manifold |
| `Samples~/Demo` | demo scene, importable from the **Samples** tab in the Package Manager |

## Performance (Apple Silicon, Unity Editor, Release)

Sphere cut (r = 0.15) in a sphere with … triangles, median:

| Triangles | 5 k | 20 k | 82 k | 328 k |
|---|---|---|---|---|
| Cut | 1.7 ms | 2.3 ms | 4.3 ms | 10.5 ms |

On top of that comes the rebuild of the Unity mesh, which grows linearly with the mesh size (82 k triangles:
~15 ms plus ~12 ms MeshCollider cooking). For very large meshes, splitting them into chunks would be the next step.
