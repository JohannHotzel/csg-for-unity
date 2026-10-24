# CSG for Unity

Subtract convex bodies from closed meshes. After every cut the result is closed and manifold, and the cut faces
get their own material (submesh 1), so objects show an interior.

The library deliberately does nothing more: no union, no cutting with non-convex bodies, no fracturing.

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

## Performance (Apple Silicon, Unity Editor, Release)

Sphere cut (r = 0.15) in a sphere with … triangles, median:

| Triangles | 5 k | 20 k | 82 k | 328 k |
|---|---|---|---|---|
| Cut | 1.7 ms | 2.3 ms | 4.3 ms | 10.5 ms |

On top of that comes the rebuild of the Unity mesh, which grows linearly with the mesh size (82 k triangles:
~15 ms plus ~12 ms MeshCollider cooking). For very large meshes, splitting them into chunks would be the next step.
