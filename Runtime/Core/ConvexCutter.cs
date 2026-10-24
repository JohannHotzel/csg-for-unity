using System;
using System.Collections.Generic;

namespace CsgForUnity
{
    /// <summary>Face of a cutter: plane n·x = w (inside n·x &lt; w), vertices counter-clockwise as seen from outside.</summary>
    public sealed class CutterFace
    {
        public double[] N;
        public double W;
        public int[] Loop;
    }

    /// <summary>
    /// Convex cutter as a B-rep. Every edge lies between exactly two faces.
    /// Built with <see cref="FromPoints"/> (convex hull) or the primitive factories.
    /// </summary>
    public sealed class ConvexCutter
    {
        public readonly double[][] Verts;
        public readonly CutterFace[] Faces;
        /// <summary>Nb[k][i]: face across edge Loop[i] → Loop[i+1] of face k.</summary>
        public readonly int[][] Nb;
        public readonly List<int>[] VertFaces;
        public readonly double[] Min = new double[3], Max = new double[3], Center = new double[3];
        public readonly double Size;
        /// <summary>Input points (used to build substitute cutters for retries).</summary>
        public double[][] Points;

        public ConvexCutter(double[][] verts, CutterFace[] faces)
        {
            Verts = verts;
            Faces = faces;
            var ek = new Dictionary<int, int>();
            for (int k = 0; k < faces.Length; k++)
            {
                var L = faces[k].Loop;
                for (int i = 0; i < L.Length; i++) ek[L[i] * 65536 + L[(i + 1) % L.Length]] = k;
            }
            Nb = new int[faces.Length][];
            for (int k = 0; k < faces.Length; k++)
            {
                var L = faces[k].Loop;
                Nb[k] = new int[L.Length];
                for (int i = 0; i < L.Length; i++)
                {
                    if (!ek.TryGetValue(L[(i + 1) % L.Length] * 65536 + L[i], out int j)) throw new InvalidOperationException("cutter is not closed");
                    Nb[k][i] = j;
                }
            }
            VertFaces = new List<int>[verts.Length];
            for (int i = 0; i < verts.Length; i++) VertFaces[i] = new List<int>();
            for (int k = 0; k < faces.Length; k++) foreach (int a in faces[k].Loop) VertFaces[a].Add(k);
            for (int d = 0; d < 3; d++) { Min[d] = double.PositiveInfinity; Max[d] = double.NegativeInfinity; }
            foreach (var v in verts)
                for (int d = 0; d < 3; d++)
                {
                    Min[d] = Math.Min(Min[d], v[d]); Max[d] = Math.Max(Max[d], v[d]);
                    Center[d] += v[d] / verts.Length;
                }
            Size = Hypot(Max[0] - Min[0], Max[1] - Min[1], Max[2] - Min[2]);
        }

        public double[] FaceCenter(int k)
        {
            var c = new double[3];
            var L = Faces[k].Loop;
            foreach (int a in L) for (int d = 0; d < 3; d++) c[d] += Verts[a][d] / L.Length;
            return c;
        }

        double[][] Src => Points ?? Verts;

        /// <summary>Scaled about the center so that the nearest face moves outward by g (bounded for flat cutters).</summary>
        public ConvexCutter ScaledOut(double g)
        {
            var c = Center;
            double r = double.PositiveInfinity, R = 0;
            foreach (var f in Faces)
            {
                double d = f.W - (f.N[0] * c[0] + f.N[1] * c[1] + f.N[2] * c[2]);
                r = Math.Min(r, d); R = Math.Max(R, d);
            }
            double s = 1 + g / Math.Max(Math.Max(r, 0.2 * R), 1e-30);
            var pts = new double[Src.Length][];
            for (int i = 0; i < pts.Length; i++) { var p = Src[i]; pts[i] = new[] { c[0] + (p[0] - c[0]) * s, c[1] + (p[1] - c[1]) * s, c[2] + (p[2] - c[2]) * s }; }
            return FromPoints(pts);
        }

        public ConvexCutter Translated(int seed, double g)
        {
            var rnd = new Mulberry(seed * 104729 + 7);
            double dx = rnd.Next() - 0.5, dy = rnd.Next() - 0.5, dz = rnd.Next() - 0.5, l = Hypot(dx, dy, dz);
            if (!(l > 0)) l = 1;
            var pts = new double[Src.Length][];
            for (int i = 0; i < pts.Length; i++) { var p = Src[i]; pts[i] = new[] { p[0] + dx / l * g, p[1] + dy / l * g, p[2] + dz / l * g }; }
            return FromPoints(pts);
        }

        /// <summary>Slightly rotated and shifted copy (for retries in numerical edge cases).</summary>
        public ConvexCutter Perturbed(int seed, double amount)
        {
            var rnd = new Mulberry(seed * 7919 + 17);
            double ax = rnd.Next() - 0.5, ay = rnd.Next() - 0.5, az = rnd.Next() - 0.5, l = Hypot(ax, ay, az);
            if (!(l > 0)) l = 1;
            double ang = amount / Math.Max(Size, 1e-12) * (0.5 + rnd.Next());
            var R = Rot.Axis(new[] { ax / l, ay / l, az / l }, ang);
            double s0 = (rnd.Next() - 0.5) * amount, s1 = (rnd.Next() - 0.5) * amount, s2 = (rnd.Next() - 0.5) * amount;
            var c = Center;
            var pts = new double[Src.Length][];
            for (int i = 0; i < pts.Length; i++)
            {
                var p = Src[i];
                double qx = p[0] - c[0], qy = p[1] - c[1], qz = p[2] - c[2];
                pts[i] = new[]
                {
                    c[0] + R[0] * qx + R[1] * qy + R[2] * qz + s0,
                    c[1] + R[3] * qx + R[4] * qy + R[5] * qz + s1,
                    c[2] + R[6] * qx + R[7] * qy + R[8] * qz + s2,
                };
            }
            return FromPoints(pts);
        }

        // ---------- convex hull (incremental, for small point sets), coplanar triangles merged ----------

        sealed class HFace { public int A, B, C; public double[] N; public double W; public bool Alive = true; }

        public static ConvexCutter FromPoints(IList<double[]> points)
        {
            int n = points.Count;
            if (n < 4) throw new ArgumentException("too few points");
            if (n > 65535) throw new ArgumentException("too many points");
            // work relative to the centroid (cutters far from the origin would lose precision otherwise)
            var o = new double[3];
            foreach (var p in points) { o[0] += p[0] / n; o[1] += p[1] / n; o[2] += p[2] / n; }
            var P = new double[n][];
            for (int i = 0; i < n; i++) P[i] = new[] { points[i][0] - o[0], points[i][1] - o[1], points[i][2] - o[2] };
            double size = 0, big = 0;
            foreach (var p in P) size = Math.Max(size, Math.Max(Math.Abs(p[0] - P[0][0]), Math.Max(Math.Abs(p[1] - P[0][1]), Math.Abs(p[2] - P[0][2]))));
            foreach (var p in points) big = Math.Max(big, Math.Max(Math.Abs(p[0]), Math.Max(Math.Abs(p[1]), Math.Abs(p[2]))));
            // tolerance: relative to the size, but never below the rounding noise of the input coordinates
            double eps = Math.Max(Math.Max(1e-10 * size, 1e-14 * big), 1e-300);

            // initial tetrahedron
            int i0 = 0;
            for (int i = 1; i < n; i++) if (P[i][0] < P[i0][0]) i0 = i;
            int i1 = -1; double best = -1;
            for (int i = 0; i < n; i++) { double d = Dist2(P[i], P[i0]); if (d > best) { best = d; i1 = i; } }
            int i2 = -1; best = -1;
            for (int i = 0; i < n; i++) { var c = Cross(Sub(P[i1], P[i0]), Sub(P[i], P[i0])); double d = Dot(c, c); if (d > best) { best = d; i2 = i; } }
            var n012 = Cross(Sub(P[i1], P[i0]), Sub(P[i2], P[i0]));
            int i3 = -1; best = -1;
            for (int i = 0; i < n; i++) { double d = Math.Abs(Dot(n012, Sub(P[i], P[i0]))); if (d > best) { best = d; i3 = i; } }
            if (best <= eps * Math.Sqrt(Dot(n012, n012))) throw new ArgumentException("points are coplanar");

            var faces = new List<HFace>();
            var edge = new Dictionary<int, int>();
            void AddFace(int a, int b, int c)
            {
                var nn = Cross(Sub(P[b], P[a]), Sub(P[c], P[a]));
                double l = Math.Sqrt(Dot(nn, nn));
                var f = new HFace { A = a, B = b, C = c, N = new[] { nn[0] / l, nn[1] / l, nn[2] / l } };
                f.W = Dot(f.N, P[a]);
                int id = faces.Count; faces.Add(f);
                edge[a * 65536 + b] = id; edge[b * 65536 + c] = id; edge[c * 65536 + a] = id;
            }
            if (Dot(n012, Sub(P[i3], P[i0])) > 0) { AddFace(i0, i2, i1); AddFace(i0, i1, i3); AddFace(i1, i2, i3); AddFace(i2, i0, i3); }
            else { AddFace(i0, i1, i2); AddFace(i0, i3, i1); AddFace(i1, i3, i2); AddFace(i2, i3, i0); }

            var used = new HashSet<int> { i0, i1, i2, i3 };
            var vis = new List<int>();
            var vset = new HashSet<int>();
            var horizon = new List<(int, int)>();
            for (int p = 0; p < n; p++)
            {
                if (used.Contains(p)) continue;
                vis.Clear();
                for (int f = 0; f < faces.Count; f++) if (faces[f].Alive && Dot(faces[f].N, P[p]) - faces[f].W > eps) vis.Add(f);
                if (vis.Count == 0) continue;
                vset.Clear(); foreach (int f in vis) vset.Add(f);
                horizon.Clear();
                foreach (int f in vis)
                {
                    var F = faces[f];
                    foreach (var (a, b) in new[] { (F.A, F.B), (F.B, F.C), (F.C, F.A) })
                        if (!edge.TryGetValue(b * 65536 + a, out int g) || !vset.Contains(g)) horizon.Add((a, b));
                }
                foreach (int f in vis)
                {
                    var F = faces[f]; F.Alive = false;
                    foreach (var (a, b) in new[] { (F.A, F.B), (F.B, F.C), (F.C, F.A) })
                        if (edge.TryGetValue(a * 65536 + b, out int g) && g == f) edge.Remove(a * 65536 + b);
                }
                foreach (var (a, b) in horizon) AddFace(a, b, p);
            }

            // merge coplanar neighboring triangles
            var alive = new List<int>();
            for (int i = 0; i < faces.Count; i++) if (faces[i].Alive) alive.Add(i);
            var par = new Dictionary<int, int>();
            foreach (int i in alive) par[i] = i;
            int Find(int i) { while (par[i] != i) { par[i] = par[par[i]]; i = par[i]; } return i; }
            foreach (int f in alive)
            {
                var F = faces[f];
                foreach (var (a, b) in new[] { (F.A, F.B), (F.B, F.C), (F.C, F.A) })
                {
                    if (!edge.TryGetValue(b * 65536 + a, out int g) || !faces[g].Alive) throw new InvalidOperationException("hull is not closed");
                    var G = faces[g];
                    if (Dot(F.N, G.N) > 1 - 1e-9 && Math.Abs(F.W - G.W) < eps * 10) par[Find(f)] = Find(g);
                }
            }
            var groups = new Dictionary<int, List<int>>();
            var order = new List<int>();
            foreach (int f in alive)
            {
                int r = Find(f);
                if (!groups.TryGetValue(r, out var L)) { groups[r] = L = new List<int>(); order.Add(r); }
                L.Add(f);
            }
            var polys = new List<List<int>>();
            foreach (int r in order)
            {
                var de = new HashSet<int>();
                var list = new List<(int, int)>();
                foreach (int f in groups[r])
                {
                    var F = faces[f];
                    foreach (var e in new[] { (F.A, F.B), (F.B, F.C), (F.C, F.A) }) { de.Add(e.Item1 * 65536 + e.Item2); list.Add(e); }
                }
                var succ = new Dictionary<int, int>();
                int start = -1;
                foreach (var (a, b) in list) if (!de.Contains(b * 65536 + a)) { succ[a] = b; if (start < 0) start = a; }
                var loop = new List<int> { start };
                for (int v = succ[start]; v != start; v = succ[v])
                {
                    loop.Add(v);
                    if (loop.Count > succ.Count) throw new InvalidOperationException("hull: face is not simple");
                }
                if (loop.Count != succ.Count) throw new InvalidOperationException("hull: face has a hole");
                polys.Add(loop);
            }
            // drop vertices with only two faces (lying on an edge)
            var deg = new Dictionary<int, int>();
            foreach (var L in polys) foreach (int a in L) deg[a] = deg.TryGetValue(a, out int c) ? c + 1 : 1;
            var remap = new Dictionary<int, int>();
            var verts = new List<double[]>();
            var rel = new List<double[]>();
            foreach (var L in polys)
                foreach (int a in L)
                    if (deg[a] >= 3 && !remap.ContainsKey(a)) { remap[a] = verts.Count; verts.Add((double[])points[a].Clone()); rel.Add(P[a]); }
            var outFaces = new CutterFace[polys.Count];
            for (int k = 0; k < polys.Count; k++)
            {
                var loopL = new List<int>();
                foreach (int a in polys[k]) if (deg[a] >= 3) loopL.Add(remap[a]);
                var loop = loopL.ToArray();
                // Newell normal
                double nx = 0, ny = 0, nz = 0;
                for (int i = 0; i < loop.Length; i++)
                {
                    var p = rel[loop[i]]; var q = rel[loop[(i + 1) % loop.Length]];
                    nx += (p[1] - q[1]) * (p[2] + q[2]); ny += (p[2] - q[2]) * (p[0] + q[0]); nz += (p[0] - q[0]) * (p[1] + q[1]);
                }
                double l = Hypot(nx, ny, nz);
                var nn = new[] { nx / l, ny / l, nz / l };
                double w = Dot(nn, o);
                foreach (int a in loop) w += Dot(nn, rel[a]) / loop.Length;
                outFaces[k] = new CutterFace { N = nn, W = w, Loop = loop };
            }
            var C = new ConvexCutter(verts.ToArray(), outFaces);
            C.Points = new double[n][];
            for (int i = 0; i < n; i++) C.Points[i] = (double[])points[i].Clone();
            return C;
        }

        // ---------- primitives: points in the local frame, rotated by R (3×3 row-major, null = no rotation) and translated ----------

        static double[][] Place(List<double[]> pts, double[] c, double[] R)
        {
            var o = new double[pts.Count][];
            for (int i = 0; i < pts.Count; i++)
            {
                var p = pts[i];
                o[i] = R == null
                    ? new[] { c[0] + p[0], c[1] + p[1], c[2] + p[2] }
                    : new[] { c[0] + R[0] * p[0] + R[1] * p[1] + R[2] * p[2], c[1] + R[3] * p[0] + R[4] * p[1] + R[5] * p[2], c[2] + R[6] * p[0] + R[7] * p[1] + R[8] * p[2] };
            }
            return o;
        }

        public static ConvexCutter Box(double[] center, double[] half, double[] R = null)
        {
            var pts = new List<double[]>();
            foreach (int sx in new[] { -1, 1 }) foreach (int sy in new[] { -1, 1 }) foreach (int sz in new[] { -1, 1 })
                pts.Add(new[] { sx * half[0], sy * half[1], sz * half[2] });
            return FromPoints(Place(pts, center, R));
        }

        /// <summary>Icosphere: detail 0 = 20 faces, 1 = 80, 2 = 320.</summary>
        public static ConvexCutter Sphere(double[] center, double r, int detail = 1, double[] R = null)
        {
            var (V, _) = IcoPoints(detail);
            var pts = new List<double[]>();
            foreach (var v in V) pts.Add(new[] { v[0] * r, v[1] * r, v[2] * r });
            return FromPoints(Place(pts, center, R));
        }

        /// <summary>Cylinder along the local y axis.</summary>
        public static ConvexCutter Cylinder(double[] center, double r, double halfH, int segs = 16, double[] R = null)
        {
            var pts = new List<double[]>();
            for (int i = 0; i < segs; i++)
            {
                double a = (double)i / segs * Math.PI * 2;
                pts.Add(new[] { r * Math.Cos(a), -halfH, r * Math.Sin(a) });
                pts.Add(new[] { r * Math.Cos(a), halfH, r * Math.Sin(a) });
            }
            return FromPoints(Place(pts, center, R));
        }

        /// <summary>Random rock: points on a dented ellipsoid.</summary>
        public static ConvexCutter Rock(double[] center, double r, int count, int seed, double[] R = null)
        {
            var rnd = new Mulberry(seed);
            var pts = new List<double[]>();
            for (int i = 0; i < count; i++)
            {
                double u = rnd.Next() * 2 - 1, a = rnd.Next() * Math.PI * 2, s = Math.Sqrt(1 - u * u), k = r * (0.65 + 0.35 * rnd.Next());
                pts.Add(new[] { k * s * Math.Cos(a), k * u * 0.8, k * s * Math.Sin(a) });
            }
            return FromPoints(Place(pts, center, R));
        }

        public static (List<double[]> V, List<int[]> F) IcoPoints(int detail)
        {
            double t = (1 + Math.Sqrt(5)) / 2;
            var V = new List<double[]>
            {
                new[] { -1, t, 0.0 }, new[] { 1, t, 0.0 }, new[] { -1, -t, 0.0 }, new[] { 1, -t, 0.0 },
                new[] { 0.0, -1, t }, new[] { 0.0, 1, t }, new[] { 0.0, -1, -t }, new[] { 0.0, 1, -t },
                new[] { t, 0, -1.0 }, new[] { t, 0, 1.0 }, new[] { -t, 0, -1.0 }, new[] { -t, 0, 1.0 },
            };
            var F = new List<int[]>
            {
                new[] { 0, 11, 5 }, new[] { 0, 5, 1 }, new[] { 0, 1, 7 }, new[] { 0, 7, 10 }, new[] { 0, 10, 11 },
                new[] { 1, 5, 9 }, new[] { 5, 11, 4 }, new[] { 11, 10, 2 }, new[] { 10, 7, 6 }, new[] { 7, 1, 8 },
                new[] { 3, 9, 4 }, new[] { 3, 4, 2 }, new[] { 3, 2, 6 }, new[] { 3, 6, 8 }, new[] { 3, 8, 9 },
                new[] { 4, 9, 5 }, new[] { 2, 4, 11 }, new[] { 6, 2, 10 }, new[] { 8, 6, 7 }, new[] { 9, 8, 1 },
            };
            for (int i = 0; i < V.Count; i++) { double l = Hypot(V[i][0], V[i][1], V[i][2]); V[i] = new[] { V[i][0] / l, V[i][1] / l, V[i][2] / l }; }
            for (int d = 0; d < detail; d++)
            {
                var mid = new Dictionary<int, int>();
                var F2 = new List<int[]>();
                int M(int a, int b)
                {
                    int k = Math.Min(a, b) * 65536 + Math.Max(a, b);
                    if (!mid.TryGetValue(k, out int id))
                    {
                        double x = V[a][0] + V[b][0], y = V[a][1] + V[b][1], z = V[a][2] + V[b][2], l = Hypot(x, y, z);
                        id = V.Count; mid[k] = id; V.Add(new[] { x / l, y / l, z / l });
                    }
                    return id;
                }
                foreach (var f in F)
                {
                    int a = f[0], b = f[1], c = f[2], ab = M(a, b), bc = M(b, c), ca = M(c, a);
                    F2.Add(new[] { a, ab, ca }); F2.Add(new[] { b, bc, ab }); F2.Add(new[] { c, ca, bc }); F2.Add(new[] { ab, bc, ca });
                }
                F = F2;
            }
            return (V, F);
        }

        // ---------- small vector helpers ----------
        internal static double Hypot(double x, double y, double z) => Math.Sqrt(x * x + y * y + z * z);
        static double[] Sub(double[] a, double[] b) => new[] { a[0] - b[0], a[1] - b[1], a[2] - b[2] };
        static double[] Cross(double[] a, double[] b) => new[] { a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] };
        static double Dot(double[] a, double[] b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        static double Dist2(double[] a, double[] b) { double x = a[0] - b[0], y = a[1] - b[1], z = a[2] - b[2]; return x * x + y * y + z * z; }
    }

    /// <summary>Small deterministic random generator (mulberry32, identical to the JS prototype).</summary>
    public sealed class Mulberry
    {
        int a;
        public Mulberry(long seed) { a = unchecked((int)seed); }
        public double Next()
        {
            unchecked
            {
                a = a + 0x6D2B79F5;
                int t = (a ^ (int)((uint)a >> 15)) * (1 | a);
                t = (t + ((t ^ (int)((uint)t >> 7)) * (61 | t))) ^ t;
                return (uint)(t ^ (int)((uint)t >> 14)) / 4294967296.0;
            }
        }
    }

    public static class Rot
    {
        /// <summary>Rotation matrix (3×3 row-major) about the unit axis a by angle ang.</summary>
        public static double[] Axis(double[] a, double ang)
        {
            double c = Math.Cos(ang), s = Math.Sin(ang), t = 1 - c, x = a[0], y = a[1], z = a[2];
            return new[]
            {
                t * x * x + c, t * x * y - s * z, t * x * z + s * y,
                t * x * y + s * z, t * y * y + c, t * y * z - s * x,
                t * x * z - s * y, t * y * z + s * x, t * z * z + c,
            };
        }
    }
}
