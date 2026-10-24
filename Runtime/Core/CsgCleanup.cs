using System;
using System.Collections.Generic;

namespace CsgForUnity
{
    /// <summary>
    /// Lokales Aufräumen nach einem Schnitt: kurze Kanten und Nadeldreiecke per Kantenkollaps entfernen,
    /// flache Nadeln (eine Ecke knapp neben der Gegenkante, keine kurze Kante) per Kanten-Flip in den Nachbarpatch.
    /// Ein Kollaps v→w wird nur gemacht, wenn
    ///  - die Topologie manifold bleibt (Link-Bedingung: gemeinsame Nachbarn von v und w sind genau die zwei Gegenecken),
    ///  - w nahe genug an der Ebene jedes Patches um v liegt (Flächen bleiben eben, Geometrie ändert sich höchstens um flatTol),
    ///  - kein Dreieck umklappt oder entartet,
    ///  - keins der neuen Dreiecke andere Teile des Netzes durchstößt (dünne Wände: flatTol kann größer als ihre Dicke sein).
    /// </summary>
    public static class CsgCleanup
    {
        public static (int collapses, int flips) Run(CsgMesh m, List<int> verts, CutOptions opts)
        {
            double diag = m.Diag > 0 ? m.Diag : 1;
            double minEdge = opts.MinEdgeRel * diag, needleEdge = opts.NeedleEdgeRel * diag, needleDeg = opts.NeedleDeg;
            double flatTol = opts.FlatTolRel * diag, sliverH = opts.SliverHRel * diag;
            var P = m.Pos; // Ecken kommen beim Aufräumen nicht hinzu; m.Tri dagegen kann wachsen → immer frisch lesen
            var queue = new List<int>(verts);
            var inQ = new HashSet<int>(verts);
            var Rv = new List<int>(); var Rw = new List<int>();
            int collapses = 0, flips = 0, guard = 0;
            double Len(int a, int b) => Math.Sqrt(Sq(P[3 * a] - P[3 * b]) + Sq(P[3 * a + 1] - P[3 * b + 1]) + Sq(P[3 * a + 2] - P[3 * b + 2]));

            void Requeue(int keep)
            {
                if (inQ.Add(keep)) queue.Add(keep);
                var T = m.Tri;
                if (m.Ring(keep, Rw))
                    foreach (int t in Rw)
                        for (int j = 0; j < 3; j++) { int x = T[3 * t + j]; if (x != keep && inQ.Add(x)) queue.Add(x); }
            }

            while (queue.Count > 0 && guard++ < 50 * verts.Count + 100)
            {
                int v = queue[queue.Count - 1];
                queue.RemoveAt(queue.Count - 1);
                inQ.Remove(v);
                if (m.VCount[v] == 0) continue;
                if (!m.Ring(v, Rv)) continue;
                // Kandidaten: kurze Kanten an v, kürzeste Kante von Nadeldreiecken an v
                int best = -1;
                double bestL = double.PositiveInfinity;
                var T = m.Tri;
                foreach (int t in Rv)
                {
                    int a = T[3 * t], b = T[3 * t + 1], c = T[3 * t + 2];
                    int j = a == v ? 0 : b == v ? 1 : 2;
                    int w1 = T[3 * t + (j + 1) % 3], w2 = T[3 * t + (j + 2) % 3];
                    double l1 = Len(v, w1), l2 = Len(v, w2);
                    if (l1 < minEdge && l1 < bestL) { best = w1; bestL = l1; }
                    if (l2 < minEdge && l2 < bestL) { best = w2; bestL = l2; }
                    if (CsgAnalyze.MinAngle(P, a, b, c) < needleDeg)
                    {
                        // kürzeste Kante des Dreiecks, wenn sie an v hängt
                        double l3 = Len(w1, w2);
                        double mn = Math.Min(l1, Math.Min(l2, l3));
                        if (mn < needleEdge && mn < l3 + 1e-300)
                        {
                            int w = l1 <= l2 ? w1 : w2;
                            double l = Math.Min(l1, l2);
                            if (l < bestL) { best = w; bestL = l; }
                        }
                    }
                }
                if (best >= 0)
                {
                    int w = best;
                    bool done = Collapse(m, v, w, flatTol, Rv, Rw);
                    int keep = w;
                    if (!done && m.Ring(w, Rw)) { done = Collapse(m, w, v, flatTol, Rw, Rv); keep = v; }
                    if (done) { collapses++; Requeue(keep); continue; }
                    if (!m.Ring(v, Rv)) continue;
                }
                // flache Nadeln an v: Gegenkante der fast-kollinearen Ecke in den Nachbarpatch flippen
                T = m.Tri;
                foreach (int t in Rv)
                {
                    bool done = false;
                    for (int j = 0; j < 3 && !done; j++)
                    {
                        int a = T[3 * t + j], b = T[3 * t + (j + 1) % 3], c = T[3 * t + (j + 2) % 3];
                        if (Height(P, a, b, c) < sliverH && FlipAcross(m, t, a, b, c, flatTol)) { flips++; Requeue(c); done = true; }
                    }
                    if (done) break;
                }
            }
            return (collapses, flips);
        }

        static double Sq(double x) => x * x;

        // Abstand von c zur Geraden durch a, b – nur wenn c zwischen a und b liegt (sonst ist es eine kurze Kante, kein Flip-Fall)
        static double Height(double[] P, int a, int b, int c)
        {
            double ux = P[3 * b] - P[3 * a], uy = P[3 * b + 1] - P[3 * a + 1], uz = P[3 * b + 2] - P[3 * a + 2];
            double vx = P[3 * c] - P[3 * a], vy = P[3 * c + 1] - P[3 * a + 1], vz = P[3 * c + 2] - P[3 * a + 2];
            double l = Math.Sqrt(ux * ux + uy * uy + uz * uz);
            double t = (ux * vx + uy * vy + uz * vz) / (l * l);
            if (!(t > 0.02 && t < 0.98)) return double.PositiveInfinity;
            return Math.Sqrt(Sq(uy * vz - uz * vy) + Sq(uz * vx - ux * vz) + Sq(ux * vy - uy * vx)) / l;
        }

        // Dreieck t = (a,b,c) mit c fast auf a-b: Kante a-b mit dem Nachbarn N = (b,a,d) flippen → (b,c,d), (c,a,d) im Patch von N
        static bool FlipAcross(CsgMesh m, int t, int a, int b, int c, double flatTol)
        {
            var T = m.Tri; var P = m.Pos;
            if (!m.Edges.TryGetValue(CsgMesh.EKey(b, a), out int N)) return false;
            int d = -1;
            for (int j = 0; j < 3; j++) { int x = T[3 * N + j]; if (x != a && x != b) d = x; }
            if (d < 0 || d == c) return false;
            if (m.Edges.ContainsKey(CsgMesh.EKey(c, d)) || m.Edges.ContainsKey(CsgMesh.EKey(d, c))) return false;
            int pN = m.TPatch[N];
            var pa = m.Patches[pN];
            if (Math.Abs(pa.N[0] * P[3 * c] + pa.N[1] * P[3 * c + 1] + pa.N[2] * P[3 * c + 2] - pa.W) > flatTol) return false;
            // nur flippen, wenn die kleinste Höhe deutlich besser wird (sonst Hin-und-her-Flips)
            double before = Math.Min(MinHeight(P, a, b, c), MinHeight(P, b, a, d));
            if (Math.Min(MinHeight(P, b, c, d), MinHeight(P, c, a, d)) < 4 * before) return false;
            if (!FlipTriOk(m, pa, b, c, d) || !FlipTriOk(m, pa, c, a, d)) return false;
            var ign = new HashSet<int> { t, N };
            if (CsgAnalyze.TriHitsMesh(m, b, c, d, ign) || CsgAnalyze.TriHitsMesh(m, c, a, d, ign)) return false;
            m.RemoveTri(t); m.RemoveTri(N);
            m.AddTri(b, c, d, pN); m.AddTri(c, a, d, pN);
            return true;
        }

        static bool FlipTriOk(CsgMesh m, Patch pa, int x, int y, int z)
        {
            var P = m.Pos;
            TriNormal(P, x, y, z, out double nx, out double ny, out double nz);
            double l = Math.Sqrt(nx * nx + ny * ny + nz * nz);
            if (!(l > 0) || pa.N[0] * nx + pa.N[1] * ny + pa.N[2] * nz < 0.7 * l) return false;
            double lmax = Math.Max(Len3(P, x, y), Math.Max(Len3(P, y, z), Len3(P, z, x)));
            return l >= 10 * m.Eps * lmax;
        }

        // v auf w ziehen; Rv = Ring von v (wird überschrieben)
        static bool Collapse(CsgMesh m, int v, int w, double flatTol, List<int> Rv, List<int> tmp)
        {
            var T = m.Tri; var P = m.Pos;
            if (!m.Ring(v, Rv)) return false;
            var ring = new List<int>(Rv);
            if (!m.Ring(w, tmp)) return false;
            var ringW = new List<int>(tmp);
            if (ring.Count <= 3 || ringW.Count <= 3) return false;
            if (!m.Edges.TryGetValue(CsgMesh.EKey(v, w), out int t1) || !m.Edges.TryGetValue(CsgMesh.EKey(w, v), out int t2)) return false;
            int Third(int t, int a, int b) { for (int j = 0; j < 3; j++) { int x = T[3 * t + j]; if (x != a && x != b) return x; } return -1; }
            int o1 = Third(t1, v, w), o2 = Third(t2, v, w);
            if (o1 == o2) return false;
            // Link-Bedingung
            var Nv = new HashSet<int>(); var Nw = new HashSet<int>();
            foreach (int t in ring) for (int j = 0; j < 3; j++) Nv.Add(T[3 * t + j]);
            foreach (int t in ringW) for (int j = 0; j < 3; j++) Nw.Add(T[3 * t + j]);
            int common = 0;
            foreach (int x in Nv)
                if (x != v && x != w && Nw.Contains(x)) { if (x != o1 && x != o2) return false; common++; }
            if (common != 2) return false;
            // keine gegenläufigen Flächen an v und w (sonst klappt eine dünne Wand zu einem Riss der Dicke null zusammen)
            foreach (int t in ring)
            {
                var n1 = m.Patches[m.TPatch[t]].N;
                foreach (int s in ringW)
                {
                    var n2 = m.Patches[m.TPatch[s]].N;
                    if (n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2] < -0.7) return false;
                }
            }
            // Geometrie
            double wx = P[3 * w], wy = P[3 * w + 1], wz = P[3 * w + 2];
            double eps = m.Eps, maxDev = 0;
            var repl = new List<int>();
            foreach (int t in ring)
            {
                if (t == t1 || t == t2) continue;
                var pa = m.Patches[m.TPatch[t]];
                double dev = Math.Abs(pa.N[0] * wx + pa.N[1] * wy + pa.N[2] * wz - pa.W);
                if (dev > flatTol) return false;
                if (dev > maxDev) maxDev = dev;
                int c0 = T[3 * t], c1 = T[3 * t + 1], c2 = T[3 * t + 2];
                int d0 = c0 == v ? w : c0, d1 = c1 == v ? w : c1, d2 = c2 == v ? w : c2;
                TriNormal(P, c0, c1, c2, out double ox, out double oy, out double oz);
                TriNormal(P, d0, d1, d2, out double nx, out double ny, out double nz);
                double lNew = Math.Sqrt(nx * nx + ny * ny + nz * nz);
                if (!(lNew > 0)) return false;
                // nicht umklappen, zur Patch-Normale passen, nicht entarten
                if (ox * nx + oy * ny + oz * nz <= 0) return false;
                if (pa.N[0] * nx + pa.N[1] * ny + pa.N[2] * nz < 0.7 * lNew) return false;
                double lmax = Math.Max(Len3(P, d0, d1), Math.Max(Len3(P, d1, d2), Len3(P, d2, d0)));
                if (lNew < 10 * eps * lmax) return false;
                repl.Add(d0); repl.Add(d1); repl.Add(d2); repl.Add(m.TPatch[t]);
            }
            // liegt w auf allen Ebenen um v (bis eps), bleibt die Fläche geometrisch gleich: keine neue Durchdringung möglich
            if (maxDev > eps)
            {
                var ign = new HashSet<int>(ring);
                for (int i = 0; i < repl.Count; i += 4) if (CsgAnalyze.TriHitsMesh(m, repl[i], repl[i + 1], repl[i + 2], ign)) return false;
            }
            foreach (int t in ring) m.RemoveTri(t);
            for (int i = 0; i < repl.Count; i += 4) m.AddTri(repl[i], repl[i + 1], repl[i + 2], repl[i + 3]);
            return true;
        }

        static double MinHeight(double[] P, int a, int b, int c)
        {
            TriNormal(P, a, b, c, out double nx, out double ny, out double nz);
            return Math.Sqrt(nx * nx + ny * ny + nz * nz) / Math.Max(Len3(P, a, b), Math.Max(Len3(P, b, c), Len3(P, c, a)));
        }

        static void TriNormal(double[] P, int a, int b, int c, out double nx, out double ny, out double nz)
        {
            double ux = P[3 * b] - P[3 * a], uy = P[3 * b + 1] - P[3 * a + 1], uz = P[3 * b + 2] - P[3 * a + 2];
            double vx = P[3 * c] - P[3 * a], vy = P[3 * c + 1] - P[3 * a + 1], vz = P[3 * c + 2] - P[3 * a + 2];
            nx = uy * vz - uz * vy; ny = uz * vx - ux * vz; nz = ux * vy - uy * vx;
        }

        static double Len3(double[] P, int a, int b) => Math.Sqrt(Sq(P[3 * a] - P[3 * b]) + Sq(P[3 * a + 1] - P[3 * b + 1]) + Sq(P[3 * a + 2] - P[3 * b + 2]));
    }
}
