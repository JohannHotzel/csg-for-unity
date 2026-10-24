using System;
using System.Collections.Generic;

namespace CsgForUnity
{
    /// <summary>
    /// Planar region that triangles belong to: a source triangle or a coplanar group from the import
    /// (Kind 0, material 0), or a cut face (Kind 1, material 1). Normals and UVs are evaluated from the patch,
    /// never re-interpolated from cut to cut.
    /// </summary>
    public sealed class Patch
    {
        public readonly double[] N;
        public readonly double W;
        public readonly int Kind, Mat;
        /// <summary>2D frame of the plane with U × V = N.</summary>
        public readonly double[] U, V;
        bool hasBasis;
        double x0, y0, i00, i01, i10, i11;
        double[] nrm, uv;
        public double UvScale = 1;

        public Patch(double[] n, double w, int kind, int mat)
        {
            N = n; W = w; Kind = kind; Mat = mat;
            var a = Math.Abs(n[0]) < 0.6 ? new[] { 1.0, 0, 0 } : Math.Abs(n[1]) < 0.6 ? new[] { 0, 1.0, 0 } : new[] { 0, 0, 1.0 };
            double ux = n[1] * a[2] - n[2] * a[1], uy = n[2] * a[0] - n[0] * a[2], uz = n[0] * a[1] - n[1] * a[0];
            double l = Math.Sqrt(ux * ux + uy * uy + uz * uz);
            U = new[] { ux / l, uy / l, uz / l };
            V = new[] { n[1] * U[2] - n[2] * U[1], n[2] * U[0] - n[0] * U[2], n[0] * U[1] - n[1] * U[0] };
        }

        /// <summary>Attribute basis from a source triangle: p (9 values), normals (9), UVs (6 or null).</summary>
        public void SetBasis(double[] p, double[] normals, double[] uvs)
        {
            double X(int i) => p[i * 3] * U[0] + p[i * 3 + 1] * U[1] + p[i * 3 + 2] * U[2];
            double Y(int i) => p[i * 3] * V[0] + p[i * 3 + 1] * V[1] + p[i * 3 + 2] * V[2];
            x0 = X(0); y0 = Y(0);
            double a = X(1) - x0, b = X(2) - x0, c = Y(1) - y0, d = Y(2) - y0, det = a * d - b * c;
            i00 = d / det; i01 = -b / det; i10 = -c / det; i11 = a / det;
            nrm = normals; uv = uvs; hasBasis = true;
        }

        /// <summary>Normal (out 0..2) and UV (out 3..4) at the point.</summary>
        public void Attr(double px, double py, double pz, double[] o)
        {
            if (!hasBasis)
            {
                o[0] = N[0]; o[1] = N[1]; o[2] = N[2];
                o[3] = (px * U[0] + py * U[1] + pz * U[2]) * UvScale;
                o[4] = (px * V[0] + py * V[1] + pz * V[2]) * UvScale;
                return;
            }
            double X = px * U[0] + py * U[1] + pz * U[2] - x0, Y = px * V[0] + py * V[1] + pz * V[2] - y0;
            double l1 = i00 * X + i01 * Y, l2 = i10 * X + i11 * Y, l0 = 1 - l1 - l2;
            double nx = l0 * nrm[0] + l1 * nrm[3] + l2 * nrm[6], ny = l0 * nrm[1] + l1 * nrm[4] + l2 * nrm[7], nz = l0 * nrm[2] + l1 * nrm[5] + l2 * nrm[8];
            double l = Math.Sqrt(nx * nx + ny * ny + nz * nz);
            if (!(l > 0)) l = 1;
            o[0] = nx / l; o[1] = ny / l; o[2] = nz / l;
            if (uv != null) { o[3] = l0 * uv[0] + l1 * uv[2] + l2 * uv[4]; o[4] = l0 * uv[1] + l1 * uv[3] + l2 * uv[5]; }
            else { o[3] = X; o[4] = Y; }
        }
    }

    /// <summary>
    /// Hash for edge and cell keys. .NET hashes a long as (high ^ low): edges (u,v) and (v,u) would always collide,
    /// and nearby vertex ids massively. Fibonacci hashing spreads the keys evenly.
    /// </summary>
    public sealed class KeyCmp : IEqualityComparer<long>
    {
        public static readonly KeyCmp I = new KeyCmp();
        public bool Equals(long a, long b) => a == b;
        public int GetHashCode(long k) => unchecked((int)(((ulong)k * 0x9E3779B97F4A7C15UL) >> 32));
    }

    /// <summary>
    /// Triangle mesh for subtracting convex cutters: vertices in double precision, triangles with patch ids,
    /// directed edge map u→v ⇒ triangle, hash grid over the triangles. Freed triangle slots are reused.
    /// </summary>
    public sealed class CsgMesh
    {
        public double[] Pos = new double[3 * 1024];
        public int Nv;
        public int[] VCount = new int[1024], VTri = new int[1024];      // triangles per vertex, one of them as entry point
        internal int[] VSlot = new int[1024], VStamp = new int[1024];   // per-cut work slot (avoids a dictionary)
        internal int CutGen;
        public int[] Tri = new int[3 * 1024];
        public int[] TPatch = new int[1024];
        public int NSlots, NTri;
        readonly List<int> free = new List<int>();
        public List<Patch> Patches = new List<Patch>();
        public Dictionary<long, int> Edges = new Dictionary<long, int>(KeyCmp.I);
        public double Cell;
        Dictionary<long, List<int>> grid = new Dictionary<long, List<int>>(KeyCmp.I);
        readonly int[] gmin = { int.MaxValue, int.MaxValue, int.MaxValue }, gmax = { int.MinValue, int.MinValue, int.MinValue };
        int[] stamp = new int[1024];
        int stampId;
        /// <summary>Classification tolerance (absolute); distances and heights below it count as zero.</summary>
        public double Eps = 1e-9;
        /// <summary>Reference size (diagonal at import) for all relative tolerances.</summary>
        public double Diag = 1;

        public CsgMesh(double cell) { Cell = cell; }

        public static long EKey(int u, int v) => ((long)u << 32) | (uint)v;
        public static int EFrom(long k) => (int)(k >> 32);
        public static int ETo(long k) => (int)(k & 0xffffffffL);

        public int AddVertex(double x, double y, double z)
        {
            if (3 * Nv + 3 > Pos.Length) Array.Resize(ref Pos, Pos.Length * 2);
            if (Nv >= VCount.Length)
            {
                int n = VCount.Length * 2;
                Array.Resize(ref VCount, n); Array.Resize(ref VTri, n); Array.Resize(ref VSlot, n); Array.Resize(ref VStamp, n);
            }
            int i = Nv++;
            VCount[i] = 0; VTri[i] = -1; VStamp[i] = 0;
            Pos[3 * i] = x; Pos[3 * i + 1] = y; Pos[3 * i + 2] = z;
            return i;
        }

        /// <summary>Reset the vertex count to n (only for newly added, still unused vertices).</summary>
        internal void TruncateVertices(int n) { Nv = n; }

        public int AddTri(int a, int b, int c, int patch)
        {
            int t;
            if (free.Count > 0) { t = free[free.Count - 1]; free.RemoveAt(free.Count - 1); }
            else
            {
                t = NSlots++;
                if (t >= TPatch.Length)
                {
                    int n = TPatch.Length * 2;
                    Array.Resize(ref Tri, 3 * n); Array.Resize(ref TPatch, n); Array.Resize(ref stamp, n);
                }
            }
            Tri[3 * t] = a; Tri[3 * t + 1] = b; Tri[3 * t + 2] = c; TPatch[t] = patch;
            VCount[a]++; VCount[b]++; VCount[c]++; VTri[a] = t; VTri[b] = t; VTri[c] = t;
            Edges[EKey(a, b)] = t; Edges[EKey(b, c)] = t; Edges[EKey(c, a)] = t;
            GridOp(t, true);
            NTri++;
            return t;
        }

        public void RemoveTri(int t)
        {
            int a = Tri[3 * t], b = Tri[3 * t + 1], c = Tri[3 * t + 2];
            GridOp(t, false);
            RemoveEdge(EKey(a, b), t); RemoveEdge(EKey(b, c), t); RemoveEdge(EKey(c, a), t);
            VCount[a]--; VCount[b]--; VCount[c]--;
            Tri[3 * t] = -1;
            free.Add(t);
            NTri--;
        }

        void RemoveEdge(long k, int t) { if (Edges.TryGetValue(k, out int x) && x == t) Edges.Remove(k); }

        public bool Alive(int t) => Tri[3 * t] >= 0;

        /// <summary>Any triangle with vertex v, otherwise -1.</summary>
        public int TriOf(int v)
        {
            int t = VTri[v];
            if (t >= 0 && Tri[3 * t] >= 0 && (Tri[3 * t] == v || Tri[3 * t + 1] == v || Tri[3 * t + 2] == v)) return t;
            if (VCount[v] == 0) return -1;
            double e = Cell * 1e-6;
            var L = new List<int>();
            QueryBox(new[] { Pos[3 * v] - e, Pos[3 * v + 1] - e, Pos[3 * v + 2] - e }, new[] { Pos[3 * v] + e, Pos[3 * v + 1] + e, Pos[3 * v + 2] + e }, L);
            foreach (int s in L) if (Tri[3 * s] == v || Tri[3 * s + 1] == v || Tri[3 * s + 2] == v) { VTri[v] = s; return s; }
            return -1;
        }

        /// <summary>Triangles around v (fan over the edges); false if v is not manifold.</summary>
        public bool Ring(int v, List<int> output)
        {
            output.Clear();
            int t0 = TriOf(v);
            if (t0 < 0) return false;
            int t = t0, guard = 0;
            do
            {
                output.Add(t);
                int j = Tri[3 * t] == v ? 0 : Tri[3 * t + 1] == v ? 1 : 2;
                int w = Tri[3 * t + (j + 1) % 3];
                if (!Edges.TryGetValue(EKey(w, v), out t)) return false;
                if (++guard > 1000) return false;
            } while (t != t0);
            return output.Count == VCount[v];
        }

        // ---------- grid ----------

        static long CellKey(int ix, int iy, int iz) => ((long)(ix & 0x1FFFFF) << 42) | ((long)(iy & 0x1FFFFF) << 21) | (long)(iz & 0x1FFFFF);

        void GridOp(int t, bool add)
        {
            double s = 1 / Cell;
            int a = 3 * Tri[3 * t], b = 3 * Tri[3 * t + 1], c = 3 * Tri[3 * t + 2];
            int x0 = (int)Math.Floor(Min3(Pos[a], Pos[b], Pos[c]) * s), x1 = (int)Math.Floor(Max3(Pos[a], Pos[b], Pos[c]) * s);
            int y0 = (int)Math.Floor(Min3(Pos[a + 1], Pos[b + 1], Pos[c + 1]) * s), y1 = (int)Math.Floor(Max3(Pos[a + 1], Pos[b + 1], Pos[c + 1]) * s);
            int z0 = (int)Math.Floor(Min3(Pos[a + 2], Pos[b + 2], Pos[c + 2]) * s), z1 = (int)Math.Floor(Max3(Pos[a + 2], Pos[b + 2], Pos[c + 2]) * s);
            if (add)
            {
                if (x0 < gmin[0]) gmin[0] = x0; if (y0 < gmin[1]) gmin[1] = y0; if (z0 < gmin[2]) gmin[2] = z0;
                if (x1 > gmax[0]) gmax[0] = x1; if (y1 > gmax[1]) gmax[1] = y1; if (z1 > gmax[2]) gmax[2] = z1;
            }
            for (int ix = x0; ix <= x1; ix++) for (int iy = y0; iy <= y1; iy++) for (int iz = z0; iz <= z1; iz++)
            {
                long k = CellKey(ix, iy, iz);
                grid.TryGetValue(k, out var L);
                if (add) { if (L == null) grid[k] = L = new List<int>(4); L.Add(t); }
                else if (L != null)
                {
                    int i = L.IndexOf(t);
                    if (i >= 0) { L[i] = L[L.Count - 1]; L.RemoveAt(L.Count - 1); }
                    if (L.Count == 0) grid.Remove(k);
                }
            }
        }

        /// <summary>All live triangles whose grid cells touch the box (no duplicates, possibly a few extra).</summary>
        public List<int> QueryBox(double[] mn, double[] mx, List<int> output)
        {
            double s = 1 / Cell;
            int id = ++stampId;
            if (gmin[0] > gmax[0]) return output;
            // clamp to the occupied cell range (huge cutters would otherwise visit billions of empty cells)
            long x0 = Math.Max(gmin[0], FloorL(mn[0] * s)), x1 = Math.Min(gmax[0], FloorL(mx[0] * s));
            long y0 = Math.Max(gmin[1], FloorL(mn[1] * s)), y1 = Math.Min(gmax[1], FloorL(mx[1] * s));
            long z0 = Math.Max(gmin[2], FloorL(mn[2] * s)), z1 = Math.Min(gmax[2], FloorL(mx[2] * s));
            if (x1 < x0 || y1 < y0 || z1 < z0) return output;
            if ((x1 - x0 + 1) * (y1 - y0 + 1) * (z1 - z0 + 1) > 2L * grid.Count)
            {
                // more cells in range than occupied cells: test every registered triangle against the box
                foreach (var L in grid.Values)
                    foreach (int t in L)
                    {
                        if (stamp[t] == id) continue;
                        stamp[t] = id;
                        if (!TriOutsideBox(t, mn, mx)) output.Add(t);
                    }
                return output;
            }
            for (long ix = x0; ix <= x1; ix++) for (long iy = y0; iy <= y1; iy++) for (long iz = z0; iz <= z1; iz++)
            {
                if (!grid.TryGetValue(CellKey((int)ix, (int)iy, (int)iz), out var L)) continue;
                foreach (int t in L) if (stamp[t] != id) { stamp[t] = id; output.Add(t); }
            }
            return output;
        }

        /// <summary>Is triangle t entirely outside the box?</summary>
        public bool TriOutsideBox(int t, double[] mn, double[] mx)
        {
            for (int d = 0; d < 3; d++)
            {
                double a = Pos[3 * Tri[3 * t] + d], b = Pos[3 * Tri[3 * t + 1] + d], c = Pos[3 * Tri[3 * t + 2] + d];
                if (Max3(a, b, c) < mn[d] || Min3(a, b, c) > mx[d]) return true;
            }
            return false;
        }

        public void Bounds(double[] mn, double[] mx)
        {
            for (int d = 0; d < 3; d++) { mn[d] = double.PositiveInfinity; mx[d] = double.NegativeInfinity; }
            for (int t = 0; t < NSlots; t++)
            {
                if (Tri[3 * t] < 0) continue;
                for (int j = 0; j < 3; j++)
                {
                    int v = Tri[3 * t + j];
                    for (int d = 0; d < 3; d++) { double x = Pos[3 * v + d]; if (x < mn[d]) mn[d] = x; if (x > mx[d]) mx[d] = x; }
                }
            }
        }

        public List<int> AllVertices()
        {
            var s = new HashSet<int>();
            var o = new List<int>();
            for (int t = 0; t < NSlots; t++)
                if (Tri[3 * t] >= 0)
                    for (int j = 0; j < 3; j++) if (s.Add(Tri[3 * t + j])) o.Add(Tri[3 * t + j]);
            return o;
        }

        /// <summary>Renumber vertices (dropping unused ones), rebuild edges and grid.</summary>
        public void Compact()
        {
            var map = new int[Nv];
            for (int i = 0; i < Nv; i++) map[i] = -1;
            var P = new List<double>();
            var tris = new List<int>();
            for (int t = 0; t < NSlots; t++)
            {
                if (Tri[3 * t] < 0) continue;
                for (int j = 0; j < 3; j++)
                {
                    int v = Tri[3 * t + j];
                    if (map[v] < 0) { map[v] = P.Count / 3; P.Add(Pos[3 * v]); P.Add(Pos[3 * v + 1]); P.Add(Pos[3 * v + 2]); }
                    tris.Add(map[v]);
                }
                tris.Add(TPatch[t]);
            }
            int nv = P.Count / 3, cap = Math.Max(1024, nv * 2);
            Pos = new double[3 * cap]; for (int i = 0; i < P.Count; i++) Pos[i] = P[i];
            Nv = nv;
            VCount = new int[cap]; VTri = new int[cap]; VSlot = new int[cap]; VStamp = new int[cap]; CutGen = 0;
            for (int i = 0; i < cap; i++) VTri[i] = -1;
            int tcap = Math.Max(1024, tris.Count / 4 * 2);
            Tri = new int[3 * tcap]; TPatch = new int[tcap]; stamp = new int[tcap];
            NSlots = 0; free.Clear(); NTri = 0; Edges = new Dictionary<long, int>(KeyCmp.I); grid = new Dictionary<long, List<int>>(KeyCmp.I);
            gmin[0] = gmin[1] = gmin[2] = int.MaxValue; gmax[0] = gmax[1] = gmax[2] = int.MinValue;
            for (int i = 0; i < tris.Count; i += 4) AddTri(tris[i], tris[i + 1], tris[i + 2], tris[i + 3]);
        }

        public CsgMesh Clone()
        {
            var m = new CsgMesh(Cell)
            {
                Pos = (double[])Pos.Clone(), Nv = Nv, VCount = (int[])VCount.Clone(), VTri = (int[])VTri.Clone(),
                VSlot = new int[VSlot.Length], VStamp = new int[VStamp.Length],
                Tri = (int[])Tri.Clone(), TPatch = (int[])TPatch.Clone(), NSlots = NSlots, NTri = NTri,
                Patches = new List<Patch>(Patches), Edges = new Dictionary<long, int>(Edges, KeyCmp.I),
                stamp = new int[stamp.Length], Eps = Eps, Diag = Diag,
            };
            m.free.AddRange(free);
            m.grid = new Dictionary<long, List<int>>(grid.Count, KeyCmp.I);
            foreach (var kv in grid) m.grid[kv.Key] = new List<int>(kv.Value);
            Array.Copy(gmin, m.gmin, 3); Array.Copy(gmax, m.gmax, 3);
            return m;
        }

        // ---------- Import ----------

        /// <summary>
        /// Mesh from indexed triangles: positions (xyz per vertex), indices, optional normals (xyz) and uvs (uv) per vertex.
        /// Vertices are welded by position; coplanar neighbors with compatible attributes form one patch.
        /// triMat: material per triangle (0 outer, 1 cut face; null = all 0).
        /// </summary>
        public static CsgMesh FromIndexed(double[] positions, int[] indices, double[] normals = null, double[] uvs = null, double epsRel = 1e-7, double weldRel = 1e-9, int[] triMat = null)
        {
            int nIn = positions.Length / 3;
            var mn = new[] { double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity };
            var mx = new[] { double.NegativeInfinity, double.NegativeInfinity, double.NegativeInfinity };
            for (int i = 0; i < nIn; i++) for (int d = 0; d < 3; d++) { mn[d] = Math.Min(mn[d], positions[3 * i + d]); mx[d] = Math.Max(mx[d], positions[3 * i + d]); }
            double diag = Math.Sqrt((mx[0] - mn[0]) * (mx[0] - mn[0]) + (mx[1] - mn[1]) * (mx[1] - mn[1]) + (mx[2] - mn[2]) * (mx[2] - mn[2]));
            if (!(diag > 0)) diag = 1;
            double weld = weldRel * diag;
            // cell size from the mean edge length
            double el = 0;
            for (int t = 0; t + 2 < indices.Length; t += 3)
            {
                int a = indices[t], b = indices[t + 1];
                el += Math.Sqrt(Sq(positions[3 * a] - positions[3 * b]) + Sq(positions[3 * a + 1] - positions[3 * b + 1]) + Sq(positions[3 * a + 2] - positions[3 * b + 2]));
            }
            el /= Math.Max(1, indices.Length / 3);
            var M = new CsgMesh(Math.Max(el * 2, diag / 200)) { Eps = epsRel * diag, Diag = diag };

            var key = new Dictionary<(long, long, long), int>();
            var vid = new int[nIn];
            for (int i = 0; i < nIn; i++)
            {
                double x = positions[3 * i], y = positions[3 * i + 1], z = positions[3 * i + 2];
                var k = ((long)Math.Round(x / weld), (long)Math.Round(y / weld), (long)Math.Round(z / weld));
                if (!key.TryGetValue(k, out int v)) { v = M.AddVertex(x, y, z); key[k] = v; }
                vid[i] = v;
            }
            // triangles + planes
            var tc = new List<int[]>(); var tv = new List<int[]>(); var tn = new List<double[]>(); var tw = new List<double>(); var tm = new List<int>();
            for (int t = 0; t + 2 < indices.Length; t += 3)
            {
                int[] c = { indices[t], indices[t + 1], indices[t + 2] }, v = { vid[c[0]], vid[c[1]], vid[c[2]] };
                if (v[0] == v[1] || v[1] == v[2] || v[2] == v[0]) continue;
                double[] P = M.Pos;
                int A = 3 * v[0], B = 3 * v[1], C = 3 * v[2];
                double e1x = P[B] - P[A], e1y = P[B + 1] - P[A + 1], e1z = P[B + 2] - P[A + 2], e2x = P[C] - P[A], e2y = P[C + 1] - P[A + 1], e2z = P[C + 2] - P[A + 2];
                double nx = e1y * e2z - e1z * e2y, ny = e1z * e2x - e1x * e2z, nz = e1x * e2y - e1y * e2x, l = Math.Sqrt(nx * nx + ny * ny + nz * nz);
                if (l < 1e-14 * diag * diag) continue;
                var nn = new[] { nx / l, ny / l, nz / l };
                tc.Add(c); tv.Add(v); tn.Add(nn); tw.Add(nn[0] * P[A] + nn[1] * P[A + 1] + nn[2] * P[A + 2]); tm.Add(triMat != null ? triMat[t / 3] : 0);
            }
            int nt = tc.Count;
            // adjacency via welded edges
            var emap = new Dictionary<long, int>(KeyCmp.I);
            for (int i = 0; i < nt; i++) for (int j = 0; j < 3; j++) emap[EKey(tv[i][j], tv[i][(j + 1) % 3])] = i;
            var par = new int[nt];
            for (int i = 0; i < nt; i++) par[i] = i;
            int Find(int i) { while (par[i] != i) { par[i] = par[par[i]]; i = par[i]; } return i; }
            double tolP = 1e-9 * diag, tolA = 1e-6;
            bool AttrOk(int s, int t)
            {
                // all vertex normals equal, UVs affinely compatible
                if (normals != null)
                {
                    int b = tc[s][0];
                    foreach (int a in new[] { tc[s][0], tc[s][1], tc[s][2], tc[t][0], tc[t][1], tc[t][2] })
                        if (Math.Abs(normals[3 * a] - normals[3 * b]) > tolA || Math.Abs(normals[3 * a + 1] - normals[3 * b + 1]) > tolA || Math.Abs(normals[3 * a + 2] - normals[3 * b + 2]) > tolA) return false;
                }
                if (uvs != null)
                {
                    var p = new Patch(tn[s], tw[s], 0, 0);
                    p.SetBasis(Corners(positions, tc[s], 3), new double[] { 0, 0, 1, 0, 0, 1, 0, 0, 1 }, Corners(uvs, tc[s], 2));
                    var o = new double[5];
                    foreach (int i in tc[t])
                    {
                        p.Attr(positions[3 * i], positions[3 * i + 1], positions[3 * i + 2], o);
                        if (Math.Abs(o[3] - uvs[2 * i]) > tolA || Math.Abs(o[4] - uvs[2 * i + 1]) > tolA) return false;
                    }
                }
                return true;
            }
            for (int i = 0; i < nt; i++)
                for (int j = 0; j < 3; j++)
                {
                    if (!emap.TryGetValue(EKey(tv[i][(j + 1) % 3], tv[i][j]), out int o) || Find(o) == Find(i)) continue;
                    var sn = tn[i]; var on = tn[o];
                    if (tm[i] == tm[o] && sn[0] * on[0] + sn[1] * on[1] + sn[2] * on[2] > 1 - 1e-12 && Math.Abs(tw[i] - tw[o]) < tolP && AttrOk(i, o)) par[Find(i)] = Find(o);
                }
            var pidx = new Dictionary<int, int>();
            for (int i = 0; i < nt; i++)
            {
                int r = Find(i);
                if (!pidx.TryGetValue(r, out int p))
                {
                    var pa = new Patch(tn[r], tw[r], tm[r] > 0 ? 1 : 0, tm[r]);
                    var nrm = normals != null ? Corners(normals, tc[r], 3) : new[] { tn[r][0], tn[r][1], tn[r][2], tn[r][0], tn[r][1], tn[r][2], tn[r][0], tn[r][1], tn[r][2] };
                    pa.SetBasis(Corners(positions, tc[r], 3), nrm, uvs != null ? Corners(uvs, tc[r], 2) : null);
                    p = M.Patches.Count; M.Patches.Add(pa); pidx[r] = p;
                }
                M.AddTri(tv[i][0], tv[i][1], tv[i][2], p);
            }
            return M;
        }

        static double[] Corners(double[] a, int[] c, int k)
        {
            var o = new double[3 * k];
            for (int i = 0; i < 3; i++) for (int d = 0; d < k; d++) o[i * k + d] = a[k * c[i] + d];
            return o;
        }

        static double Sq(double x) => x * x;
        static long FloorL(double x) => x < -9e18 ? long.MinValue / 4 : x > 9e18 ? long.MaxValue / 4 : (long)Math.Floor(x);
        internal static double Min3(double a, double b, double c) => a < b ? (a < c ? a : c) : (b < c ? b : c);
        internal static double Max3(double a, double b, double c) => a > b ? (a > c ? a : c) : (b > c ? b : c);
    }
}
