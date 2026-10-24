using System;
using System.Collections.Generic;

namespace CsgForUnity
{
    /// <summary>
    /// Triangulates a planar region given by directed boundary edges (the region left of the edges).
    /// Unlike earcut, boundary points are never dropped: every boundary vertex stays a triangle corner, otherwise
    /// T-junctions with the neighboring faces would appear. Nearly collinear points are taken out temporarily and
    /// reinserted afterwards by splitting edges; finally Delaunay flips in the interior.
    /// Holes are bridged to the outer contour as in earcut (Mapbox, ISC).
    /// </summary>
    public static class Triangulator
    {
        sealed class Node
        {
            public int I; public double X, Y; public Node Prev, Next;
            public Node(int i, double x, double y) { I = i; X = x; Y = y; }
        }

        static double Cross(Node a, Node b, Node c) => (b.X - a.X) * (c.Y - a.Y) - (b.Y - a.Y) * (c.X - a.X);

        /// <summary>
        /// edges: [a0,b0, a1,b1, …] vertex ids; pos: xyz per vertex; (pu,pv): 2D frame; output: [a,b,c,…].
        /// Returns null on success, otherwise an error message.
        /// </summary>
        public static string TriangulateRegion(double[] pos, List<int> edges, double[] pu, double[] pv, List<int> output, double epsLine)
        {
            int ne = edges.Count >> 1;
            if (ne == 0) return null;
            var loc = new Dictionary<int, int>();
            var X = new List<double>(); var Y = new List<double>(); var ids = new List<int>();
            double ox = pos[3 * edges[0]], oy = pos[3 * edges[0] + 1], oz = pos[3 * edges[0] + 2];
            int Li(int v)
            {
                if (!loc.TryGetValue(v, out int i))
                {
                    i = ids.Count; loc[v] = i; ids.Add(v);
                    double x = pos[3 * v] - ox, y = pos[3 * v + 1] - oy, z = pos[3 * v + 2] - oz;
                    X.Add(x * pu[0] + y * pu[1] + z * pu[2]); Y.Add(x * pv[0] + y * pv[1] + z * pv[2]);
                }
                return i;
            }
            var ea = new int[ne]; var eb = new int[ne];
            for (int e = 0; e < ne; e++) { ea[e] = Li(edges[2 * e]); eb[e] = Li(edges[2 * e + 1]); }
            int nv = ids.Count;
            var outE = new List<int>[nv];
            for (int v = 0; v < nv; v++) outE[v] = new List<int>(2);
            for (int e = 0; e < ne; e++) outE[ea[e]].Add(e);
            var inDeg = new int[nv];
            for (int e = 0; e < ne; e++) inDeg[eb[e]]++;
            for (int v = 0; v < nv; v++) if (inDeg[v] != outE[v].Count) return "boundary not closed";

            // loops; with several outgoing edges take the smallest clockwise angle
            var used = new bool[ne];
            var loops = new List<List<int>>();
            for (int e0 = 0; e0 < ne; e0++)
            {
                if (used[e0]) continue;
                var L = new List<int>();
                int e = e0, guard = 0;
                for (; ; )
                {
                    used[e] = true; L.Add(ea[e]);
                    int v = eb[e], u = ea[e];
                    var cand = outE[v];
                    int best = -1; double bestAng = double.PositiveInfinity;
                    double rx = X[u] - X[v], ry = Y[u] - Y[v];
                    foreach (int f in cand)
                    {
                        if (used[f] && f != e0) continue;
                        if (cand.Count == 1) { best = f; break; }
                        double dx = X[eb[f]] - X[v], dy = Y[eb[f]] - Y[v];
                        double a = Math.Atan2(rx * dy - ry * dx, rx * dx + ry * dy); // counter-clockwise from r to d
                        double cw = -a; if (cw <= 0) cw += 2 * Math.PI;
                        if (cw < bestAng) { bestAng = cw; best = f; }
                    }
                    if (best < 0) return "loop breaks off";
                    if (best == e0) break;
                    e = best;
                    if (++guard > ne) return "loop does not terminate";
                }
                loops.Add(L);
            }
            // outer (area > 0) / hole (< 0)
            double Area(List<int> L) { double s = 0; for (int i = 0; i < L.Count; i++) { int a = L[i], b = L[(i + 1) % L.Count]; s += X[a] * Y[b] - X[b] * Y[a]; } return s / 2; }
            var outers = new List<(List<int> L, double A, List<List<int>> H)>();
            var holes = new List<List<int>>();
            foreach (var L in loops)
            {
                if (L.Count < 3) return "degenerate loop";
                double A = Area(L);
                if (A > 0) outers.Add((L, A, new List<List<int>>())); else if (A < 0) holes.Add(L); else return "loop without area";
            }
            bool InPoly(List<int> L, double px, double py)
            {
                bool c = false;
                for (int i = 0, j = L.Count - 1; i < L.Count; j = i++)
                {
                    int a = L[i], b = L[j];
                    if ((Y[a] > py) != (Y[b] > py) && px < (X[b] - X[a]) * (py - Y[a]) / (Y[b] - Y[a]) + X[a]) c = !c;
                }
                return c;
            }
            foreach (var H in holes)
            {
                // test point just left of the longest hole edge (inside the region, between hole and outer contour)
                int bi = 0; double bl = -1;
                for (int i = 0; i < H.Count; i++) { int a0 = H[i], b0 = H[(i + 1) % H.Count]; double l = Math.Sqrt(Sq(X[b0] - X[a0]) + Sq(Y[b0] - Y[a0])); if (l > bl) { bl = l; bi = i; } }
                int a = H[bi], b = H[(bi + 1) % H.Count];
                double mx = (X[a] + X[b]) / 2, my = (Y[a] + Y[b]) / 2, dx = (X[b] - X[a]) / bl, dy = (Y[b] - Y[a]) / bl, off = bl * 1e-4;
                double px = mx - dy * off, py = my + dx * off;
                int bestO = -1;
                for (int k = 0; k < outers.Count; k++) if (InPoly(outers[k].L, px, py) && (bestO < 0 || outers[k].A < outers[bestO].A)) bestO = k;
                if (bestO < 0) return "hole without outer contour";
                outers[bestO].H.Add(H);
            }
            var Xa = X.ToArray(); var Ya = Y.ToArray();
            var tris = new List<int[]>();
            foreach (var O in outers)
            {
                // graded: simplify points collinear within eps, then only exactly collinear ones, then none
                string r = null;
                foreach (double simp in new[] { epsLine, epsLine * 1e-4, 0 })
                {
                    int n0 = tris.Count;
                    r = EarcutPoly(O.L, O.H, Xa, Ya, epsLine, tris, simp);
                    if (r == null) break;
                    tris.RemoveRange(n0, tris.Count - n0);
                }
                if (r != null) return r;
            }
            DelaunayFlips(tris, Xa, Ya, ea, eb, epsLine);
            foreach (var t in tris) { output.Add(ids[t[0]]); output.Add(ids[t[1]]); output.Add(ids[t[2]]); }
            return null;
        }

        static Node Linked(List<int> L, double[] X, double[] Y)
        {
            Node first = null, last = null;
            foreach (int i in L)
            {
                var n = new Node(i, X[i], Y[i]);
                if (first == null) first = n; else { last.Next = n; n.Prev = last; }
                last = n;
            }
            last.Next = first; first.Prev = last;
            return first;
        }

        static string EarcutPoly(List<int> outerL, List<List<int>> holesL, double[] X, double[] Y, double epsLine, List<int[]> tris, double simpEps)
        {
            // take out nearly collinear boundary points (remembering where they were), reinsert them by edge splits afterwards
            var removed = new List<int>();
            var count = new Dictionary<int, int>();
            void Count(List<int> L) { foreach (int i in L) count[i] = count.TryGetValue(i, out int c) ? c + 1 : 1; }
            Count(outerL); foreach (var H in holesL) Count(H);
            var oL = simpEps > 0 ? SimplifyLoop(outerL, X, Y, simpEps, count, removed) : new List<int>(outerL);
            var hL = new List<List<int>>();
            foreach (var H in holesL) hL.Add(simpEps > 0 ? SimplifyLoop(H, X, Y, simpEps, count, removed) : new List<int>(H));
            if (oL.Count < 3) return "region without area";
            foreach (var H in hL) if (H.Count < 3) return "region without area";
            var outer = Linked(oL, X, Y);
            if (hL.Count > 0)
            {
                var hs = new List<Node>();
                foreach (var H in hL)
                {
                    Node n = Linked(H, X, Y), left = n, p = n;
                    do { if (p.X < left.X || (p.X == left.X && p.Y < left.Y)) left = p; p = p.Next; } while (p != n);
                    hs.Add(left);
                }
                hs.Sort((a, b) => a.X.CompareTo(b.X));
                foreach (var h in hs)
                {
                    var m = FindHoleBridge(h, outer);
                    if (m == null) return "no bridge to hole";
                    SplitPolygon(m, h);
                }
            }
            int t0 = tris.Count;
            string r = EarClip(outer, epsLine, tris, 0);
            if (r != null) return r;
            if (removed.Count > 0) return Reinsert(tris, t0, removed, X, Y);
            return null;
        }

        // remove points b with |height above a-c| < eps lying between a and c (repeatedly, neighbors change)
        static List<int> SimplifyLoop(List<int> L, double[] X, double[] Y, double eps, Dictionary<int, int> count, List<int> removed)
        {
            var R = new List<int>(L);
            bool changed = true;
            while (changed && R.Count > 3)
            {
                changed = false;
                for (int j = 0; j < R.Count && R.Count > 3; j++)
                {
                    int a = R[(j + R.Count - 1) % R.Count], b = R[j], c = R[(j + 1) % R.Count];
                    if (count[b] > 1 || a == c) continue;
                    double dx = X[c] - X[a], dy = Y[c] - Y[a], l2 = dx * dx + dy * dy;
                    if (!(l2 > 0)) continue;
                    double bx = X[b] - X[a], by = Y[b] - Y[a];
                    double t = (bx * dx + by * dy) / l2;
                    if (t <= 0 || t >= 1) continue;
                    if (Math.Abs(dx * by - dy * bx) / Math.Sqrt(l2) < eps) { removed.Add(b); removed.Add(a); removed.Add(c); R.RemoveAt(j); j--; changed = true; }
                }
            }
            return R;
        }

        // reinsert removed points in reverse order: the triangle with boundary edge a->c is split into (a,b,x) and (b,c,x)
        static string Reinsert(List<int[]> tris, int t0, List<int> removed, double[] X, double[] Y)
        {
            var em = new Dictionary<long, int>(KeyCmp.I);
            long Key(int a, int b) => ((long)a << 32) | (uint)b;
            for (int i = t0; i < tris.Count; i++) { var t = tris[i]; em[Key(t[0], t[1])] = i; em[Key(t[1], t[2])] = i; em[Key(t[2], t[0])] = i; }
            double O2(int p, int q, int r) => (X[q] - X[p]) * (Y[r] - Y[p]) - (Y[q] - Y[p]) * (X[r] - X[p]);
            for (int r = removed.Count - 3; r >= 0; r -= 3)
            {
                int b = removed[r], a = removed[r + 1], c = removed[r + 2];
                if (!em.TryGetValue(Key(a, c), out int i)) return "boundary point cannot be reinserted";
                var t = tris[i];
                int j = t[0] == a ? 0 : t[1] == a ? 1 : 2, x = t[(j + 2) % 3];
                // both new triangles must stay counter-clockwise (b lies on a-c only up to eps)
                if (!(O2(a, b, x) > 0) || !(O2(b, c, x) > 0)) return "boundary point flips a triangle";
                for (int k = 0; k < 3; k++) em.Remove(Key(t[k], t[(k + 1) % 3]));
                tris[i] = new[] { a, b, x };
                tris.Add(new[] { b, c, x });
                int n = tris.Count - 1;
                em[Key(a, b)] = i; em[Key(b, x)] = i; em[Key(x, a)] = i;
                em[Key(b, c)] = n; em[Key(c, x)] = n; em[Key(x, b)] = n;
            }
            return null;
        }

        // convex and smallest triangle height > eps
        static bool Convex(double ax, double ay, double bx, double by, double cx, double cy, double eps)
        {
            double cr = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
            if (cr <= 0) return false;
            return cr > eps * Math.Max(Math.Sqrt(Sq(cx - ax) + Sq(cy - ay)), Math.Max(Math.Sqrt(Sq(bx - ax) + Sq(by - ay)), Math.Sqrt(Sq(cx - bx) + Sq(cy - by))));
        }
        static bool Convex(Node a, Node b, Node c, double eps) => Convex(a.X, a.Y, b.X, b.Y, c.X, c.Y, eps);

        // p inside the triangle or closer than eps to its boundary (then it blocks the ear)
        static bool InTri(Node a, Node b, Node c, Node p, double eps, double lab, double lbc, double lca) =>
            Cross(a, b, p) >= -eps * lab && Cross(b, c, p) >= -eps * lbc && Cross(c, a, p) >= -eps * lca;

        static bool IsEar(Node ear, double eps)
        {
            Node a = ear.Prev, b = ear, c = ear.Next;
            if (!Convex(a, b, c, eps)) return false;
            double lab = Math.Sqrt(Sq(b.X - a.X) + Sq(b.Y - a.Y)), lbc = Math.Sqrt(Sq(c.X - b.X) + Sq(c.Y - b.Y)), lca = Math.Sqrt(Sq(a.X - c.X) + Sq(a.Y - c.Y));
            for (Node p = c.Next; p != a; p = p.Next)
            {
                if (p.I == a.I || p.I == b.I || p.I == c.I) continue;
                if (InTri(a, b, c, p, eps, lab, lbc, lca)) return false;
            }
            return true;
        }

        static string EarClip(Node start, double eps, List<int[]> tris, int depth)
        {
            Node ear = start;
            int count = 1;
            for (Node p = start.Next; p != start; p = p.Next) count++;
            Node stop = ear;
            long guard = 0;
            while (ear.Prev != ear.Next)
            {
                if (IsEar(ear, eps))
                {
                    tris.Add(new[] { ear.Prev.I, ear.I, ear.Next.I });
                    ear.Prev.Next = ear.Next; ear.Next.Prev = ear.Prev;
                    ear = ear.Next.Next; stop = ear;
                    if (--count < 3) break;
                    continue;
                }
                ear = ear.Next;
                if (ear == stop)
                {
                    // no ear: split the polygon along a valid diagonal
                    if (depth > 8) return "ear clipping is stuck";
                    string r = SplitEarcut(ear, eps, tris, depth);
                    if (r == null) return "ear clipping is stuck";
                    return r.Length == 0 ? null : r;
                }
                if (++guard > 10000000) return "ear clipping does not terminate";
            }
            return null;
        }

        // fallback: split along a valid diagonal; bounded search (otherwise O(n³) per level for degenerate polygons).
        // returns "" = success, text = error, null = no diagonal found
        static string SplitEarcut(Node start, double eps, List<int[]> tris, int depth)
        {
            Node a = start;
            int budget = 4000;
            do
            {
                Node b = a.Next.Next;
                while (b != a.Prev)
                {
                    if (--budget < 0) return null;
                    if (a.I != b.I && IsValidDiagonal(a, b))
                    {
                        Node c = SplitPolygon(a, b);
                        string r1 = EarClip(a, eps, tris, depth + 1);
                        if (r1 != null) return r1;
                        string r2 = EarClip(c, eps, tris, depth + 1);
                        return r2 ?? OkMarker;
                    }
                    b = b.Next;
                }
                a = a.Next;
            } while (a != start);
            return null;
        }
        const string OkMarker = "";

        // --- adapted from earcut (counter-clockwise orientation, Area = -Cross) ---
        static Node FindHoleBridge(Node hole, Node outer)
        {
            Node p = outer, m = null;
            double hx = hole.X, hy = hole.Y, qx = double.NegativeInfinity;
            do
            {
                if (hy <= p.Y && hy >= p.Next.Y && p.Next.Y != p.Y)
                {
                    double x = p.X + (hy - p.Y) * (p.Next.X - p.X) / (p.Next.Y - p.Y);
                    if (x <= hx && x > qx)
                    {
                        qx = x; m = p.X < p.Next.X ? p : p.Next;
                        if (x == hx) return m;
                    }
                }
                p = p.Next;
            } while (p != outer);
            if (m == null) return null;
            Node stop = m;
            double mx = m.X, my = m.Y, tanMin = double.PositiveInfinity;
            p = m;
            do
            {
                if (hx >= p.X && p.X >= mx && hx != p.X && PointInTriangle(hy < my ? hx : qx, hy, mx, my, hy < my ? qx : hx, hy, p.X, p.Y))
                {
                    double tan = Math.Abs(hy - p.Y) / (hx - p.X);
                    if (LocallyInside(p, hole) && (tan < tanMin || (tan == tanMin && (p.X > m.X || (p.X == m.X && SectorContainsSector(m, p)))))) { m = p; tanMin = tan; }
                }
                p = p.Next;
            } while (p != stop);
            return m;
        }

        static bool PointInTriangle(double ax, double ay, double bx, double by, double cx, double cy, double px, double py) =>
            (cx - px) * (ay - py) >= (ax - px) * (cy - py) && (ax - px) * (by - py) >= (bx - px) * (ay - py) && (bx - px) * (cy - py) >= (cx - px) * (by - py);
        static double Area(Node p, Node q, Node r) => (q.Y - p.Y) * (r.X - q.X) - (q.X - p.X) * (r.Y - q.Y);
        static bool EqualsXY(Node a, Node b) => a.X == b.X && a.Y == b.Y;
        static bool SectorContainsSector(Node m, Node p) => Area(m.Prev, m, p.Prev) < 0 && Area(p.Next, m, m.Next) < 0;
        static bool LocallyInside(Node a, Node b) =>
            Area(a.Prev, a, a.Next) < 0 ? Area(a, b, a.Next) >= 0 && Area(a, a.Prev, b) >= 0 : Area(a, b, a.Prev) < 0 || Area(a, a.Next, b) < 0;
        static bool MiddleInside(Node a, Node b)
        {
            Node p = a; bool inside = false;
            double px = (a.X + b.X) / 2, py = (a.Y + b.Y) / 2;
            do
            {
                if (((p.Y > py) != (p.Next.Y > py)) && p.Next.Y != p.Y && (px < (p.Next.X - p.X) * (py - p.Y) / (p.Next.Y - p.Y) + p.X)) inside = !inside;
                p = p.Next;
            } while (p != a);
            return inside;
        }
        static int Sign(double n) => n > 0 ? 1 : n < 0 ? -1 : 0;
        static bool OnSegment(Node p, Node q, Node r) => q.X <= Math.Max(p.X, r.X) && q.X >= Math.Min(p.X, r.X) && q.Y <= Math.Max(p.Y, r.Y) && q.Y >= Math.Min(p.Y, r.Y);
        static bool Intersects(Node p1, Node q1, Node p2, Node q2)
        {
            int o1 = Sign(Area(p1, q1, p2)), o2 = Sign(Area(p1, q1, q2)), o3 = Sign(Area(p2, q2, p1)), o4 = Sign(Area(p2, q2, q1));
            if (o1 != o2 && o3 != o4) return true;
            if (o1 == 0 && OnSegment(p1, p2, q1)) return true;
            if (o2 == 0 && OnSegment(p1, q2, q1)) return true;
            if (o3 == 0 && OnSegment(p2, p1, q2)) return true;
            if (o4 == 0 && OnSegment(p2, q1, q2)) return true;
            return false;
        }
        static bool IntersectsPolygon(Node a, Node b)
        {
            Node p = a;
            do
            {
                if (p.I != a.I && p.Next.I != a.I && p.I != b.I && p.Next.I != b.I && Intersects(p, p.Next, a, b)) return true;
                p = p.Next;
            } while (p != a);
            return false;
        }
        static bool IsValidDiagonal(Node a, Node b) =>
            a.Next.I != b.I && a.Prev.I != b.I && !IntersectsPolygon(a, b) &&
            (LocallyInside(a, b) && LocallyInside(b, a) && MiddleInside(a, b) && (Area(a.Prev, a, b.Prev) != 0 || Area(a, b.Prev, b) != 0)
             || EqualsXY(a, b) && Area(a.Prev, a, a.Next) > 0 && Area(b.Prev, b, b.Next) > 0);
        static Node SplitPolygon(Node a, Node b)
        {
            Node a2 = new Node(a.I, a.X, a.Y), b2 = new Node(b.I, b.X, b.Y), an = a.Next, bp = b.Prev;
            a.Next = b; b.Prev = a;
            a2.Next = an; an.Prev = a2;
            b2.Next = a2; a2.Prev = b2;
            bp.Next = b2; b2.Prev = bp;
            return b2;
        }

        // --- Delaunay flips (boundary edges stay fixed) ---
        static void DelaunayFlips(List<int[]> tris, double[] X, double[] Y, int[] ea, int[] eb, double eps)
        {
            long Key(int a, int b) => ((long)a << 32) | (uint)b;
            var fix = new HashSet<long>(KeyCmp.I);
            for (int e = 0; e < ea.Length; e++) fix.Add(Key(Math.Min(ea[e], eb[e]), Math.Max(ea[e], eb[e])));
            var em = new Dictionary<long, int>(KeyCmp.I);
            for (int i = 0; i < tris.Count; i++) { var t = tris[i]; em[Key(t[0], t[1])] = i; em[Key(t[1], t[2])] = i; em[Key(t[2], t[0])] = i; }
            var stack = new Stack<int>();
            for (int i = 0; i < tris.Count; i++) stack.Push(i);
            int guard = 0;
            while (stack.Count > 0 && guard++ < 20 * tris.Count + 100)
            {
                int ti = stack.Pop();
                var t = tris[ti];
                for (int j = 0; j < 3; j++)
                {
                    int a = t[j], b = t[(j + 1) % 3], c = t[(j + 2) % 3];
                    if (fix.Contains(Key(Math.Min(a, b), Math.Max(a, b)))) continue;
                    if (!em.TryGetValue(Key(b, a), out int oi)) continue;
                    var o = tris[oi];
                    int d = o[0] != a && o[0] != b ? o[0] : o[1] != a && o[1] != b ? o[1] : o[2];
                    if (d == c) continue;
                    if (!InCircle(X, Y, a, b, c, d)) continue;
                    // new triangles (a,d,c) and (d,b,c) must be convex
                    if (!Convex(X[a], Y[a], X[d], Y[d], X[c], Y[c], eps) || !Convex(X[d], Y[d], X[b], Y[b], X[c], Y[c], eps)) continue;
                    foreach (var s in new[] { t, o }) { em.Remove(Key(s[0], s[1])); em.Remove(Key(s[1], s[2])); em.Remove(Key(s[2], s[0])); }
                    tris[ti] = new[] { a, d, c }; tris[oi] = new[] { d, b, c };
                    foreach (int i in new[] { ti, oi }) { var s = tris[i]; em[Key(s[0], s[1])] = i; em[Key(s[1], s[2])] = i; em[Key(s[2], s[0])] = i; }
                    stack.Push(ti); stack.Push(oi);
                    break;
                }
            }
        }

        static bool InCircle(double[] X, double[] Y, int a, int b, int c, int d)
        {
            double adx = X[a] - X[d], ady = Y[a] - Y[d], bdx = X[b] - X[d], bdy = Y[b] - Y[d], cdx = X[c] - X[d], cdy = Y[c] - Y[d];
            double ad = adx * adx + ady * ady, bd = bdx * bdx + bdy * bdy, cd = cdx * cdx + cdy * cdy;
            double det = adx * (bdy * cd - bd * cdy) - ady * (bdx * cd - bd * cdx) + ad * (bdx * cdy - bdy * cdx);
            double s = Math.Abs(adx) + Math.Abs(ady) + Math.Abs(bdx) + Math.Abs(bdy) + Math.Abs(cdx) + Math.Abs(cdy);
            return det > 1e-12 * s * s * s * s;
        }

        static double Sq(double x) => x * x;
    }
}
