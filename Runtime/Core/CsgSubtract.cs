using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace CsgForUnity
{
    [Serializable]
    public sealed class CutOptions
    {
        /// <summary>Wiederholungen mit leicht verändertem Körper, wenn ein Versuch scheitert.</summary>
        public int Retries = 6;
        /// <summary>Störung der Ersatzkörper relativ zur Netzdiagonale.</summary>
        public double PerturbRel = 2e-4;
        /// <summary>Nach dem Schnitt kurze Kanten und Nadeldreiecke entfernen.</summary>
        public bool Cleanup = true;
        public double MinEdgeRel = 1e-4, NeedleEdgeRel = 2e-3, NeedleDeg = 2, FlatTolRel = 2e-5, SliverHRel = 1e-5;
        /// <summary>UV-Maßstab der Schnittflächen (UV = Position in der Ebene × Maßstab).</summary>
        public double CapUvScale = 1;
    }

    public sealed class CutResult
    {
        public bool Ok, Changed;
        public int Retries, Removed, Added, Collapses, Flips;
        public string Reason;
        public List<string> Reasons = new List<string>();
        /// <summary>Tatsächlich verwendeter Körper (nach Wiederholungen evtl. ein Ersatzkörper).</summary>
        public ConvexCutter Cutter;
        public double Ms;
        public override string ToString() => Ok ? (Changed ? $"ok −{Removed} +{Added} Dreiecke, {Collapses} Kollapse, {Retries} Wdh., {Ms:0.00} ms" : "ok, nichts berührt") : $"abgelehnt: {Reason}";
    }

    /// <summary>
    /// Netz minus konvexer Körper.
    ///
    /// Ablauf je Versuch (nichts am Netz ändert sich, bevor alles geprüft ist):
    ///  1. Kandidaten aus dem Gitter. Jedes Dreieck T wird mit Sutherland-Hodgman an allen Ebenen geschnitten → P_T = T ∩ C.
    ///     Vorzeichen (−,0,+) je (Punkt, Ebene) werden einmal bestimmt und gemerkt; Schnittpunkte je (Kante, Ebene) gibt es
    ///     genau einmal. Benachbarte Dreiecke sehen daher dieselben Punkte und Entscheidungen. 0 zählt als innen
    ///     ("C infinitesimal vergrößert") – damit werden koplanare Fälle gültig statt zu Doppelschichten.
    ///  2. Punkte näher als eps werden zusammengelegt, Punkte auf ≥3 Ebenen sind Körperecken.
    ///  3. R_T = T \ P_T als gerichtete Kanten (Rand von T unterteilt + P_T umgekehrt, Gegenkanten heben sich auf).
    ///  4. Kanten ohne Gegenstück liegen auf ∂C → je Körperfläche k die Deckelränder. Offene Ketten werden entlang des
    ///     Flächenrands (Körperkanten) geschlossen; Flächen ohne Ketten erben "innen/außen" von Nachbarflächen.
    ///  5. Je betroffener Fläche (Patch) und je Deckel triangulieren, alles prüfen (jede Kante genau ein Gegenstück), übernehmen.
    /// Scheitert ein Schritt, wird mit leicht verändertem Körper wiederholt; das Netz wird nie halb verändert.
    /// </summary>
    public static class Csg
    {
        public static CutResult Subtract(CsgMesh mesh, ConvexCutter cutter, CutOptions opts = null)
        {
            opts ??= new CutOptions();
            var sw = Stopwatch.StartNew();
            var C = cutter;
            var res = new CutResult();
            for (int a = 0; a <= opts.Retries; a++)
            {
                int nv0 = mesh.Nv;
                Cut cut = null;
                string reason;
                try
                {
                    cut = new Cut(mesh, C, opts);
                    reason = cut.Run();
                }
                catch (Exception e) { reason = "Ausnahme: " + e.Message; }
                if (reason == null)
                {
                    res.Ok = true; res.Changed = cut.Changed; res.Removed = cut.Removed; res.Added = cut.Added;
                    res.Retries = a; res.Cutter = C;
                    if (cut.Changed && opts.Cleanup)
                    {
                        var c = CsgCleanup.Run(mesh, cut.Verts, opts);
                        res.Collapses = c.collapses; res.Flips = c.flips;
                    }
                    res.Ms = sw.Elapsed.TotalMilliseconds;
                    return res;
                }
                // gescheitert: neue Punkte wieder verwerfen
                mesh.TruncateVertices(nv0);
                res.Reasons.Add(reason); res.Reason = reason;
                // Störung deutlich über der Aufräumschwelle: erzeugt robuste, kleine Merkmale statt Beinahe-Entartungen
                try { C = Variant(cutter, a + 1, opts.PerturbRel * mesh.Diag); }
                catch (Exception e) { C = cutter; res.Reasons.Add("Ersatzkörper: " + e.Message); }
            }
            res.Retries = opts.Retries;
            res.Ms = sw.Elapsed.TotalMilliseconds;
            return res;
        }

        // Ersatzkörper für Wiederholungen: Ebenen bleiben zunächst parallel (keine keilförmigen Splitter)
        static ConvexCutter Variant(ConvexCutter C, int attempt, double g)
        {
            switch (attempt)
            {
                case 1: return C.ScaledOut(g);
                case 2: return C.ScaledOut(3 * g);
                case 3: return C.Translated(attempt, 2 * g);
                case 4: return C.Perturbed(attempt, 4 * g);
                // letzte Versuche: deutlich größer (sichtbar kaum, aber weit weg von allen Beinahe-Entartungen)
                case 5: return C.ScaledOut(10 * g);
                default: return C.Perturbed(attempt, 15 * g);
            }
        }

        sealed class Cut
        {
            readonly CsgMesh m;
            readonly ConvexCutter C;
            readonly CutOptions opts;
            readonly int K, gen, nv0;
            readonly double eps;
            readonly double[] pn, pw;
            sbyte[] sg;
            int[] p0, p1;
            int ns;
            readonly Dictionary<long, int>[] cross;
            readonly Dictionary<int, long> onEdge = new Dictionary<int, long>();
            readonly Dictionary<int, List<int>> zs = new Dictionary<int, List<int>>();
            readonly int[] curT = new int[3];
            Dictionary<int, int> alias;
            Dictionary<int, List<int>> hash;
            int[] rep;
            Dictionary<int, int> repOf;
            double raw;

            public bool Changed;
            public int Removed, Added;
            public List<int> Verts;

            public Cut(CsgMesh mesh, ConvexCutter c, CutOptions o)
            {
                m = mesh; C = c; opts = o;
                K = C.Faces.Length;
                eps = mesh.Eps;
                pn = new double[3 * K]; pw = new double[K];
                for (int k = 0; k < K; k++) { var f = C.Faces[k]; pn[3 * k] = f.N[0]; pn[3 * k + 1] = f.N[1]; pn[3 * k + 2] = f.N[2]; pw[k] = f.W; }
                gen = ++mesh.CutGen;
                sg = new sbyte[256 * K]; Fill(sg, (sbyte)2);
                p0 = new int[256]; p1 = new int[256]; Fill(p0, -1); Fill(p1, -1);
                cross = new Dictionary<long, int>[K];
                for (int k = 0; k < K; k++) cross[k] = new Dictionary<long, int>(KeyCmp.I);
                nv0 = mesh.Nv;
            }

            static void Fill<T>(T[] a, T v) { for (int i = 0; i < a.Length; i++) a[i] = v; }

            // Arbeitsplatz je Punkt; Netzecken bekommen sofort alle Vorzeichen (eine Schleife über die Ebenen)
            int SlotOf(int v)
            {
                if (m.VStamp[v] == gen) return m.VSlot[v];
                int s = ns++;
                m.VStamp[v] = gen; m.VSlot[v] = s;
                if (s >= p0.Length)
                {
                    int n = p0.Length * 2, old = p0.Length;
                    Array.Resize(ref p0, n); Array.Resize(ref p1, n); Array.Resize(ref sg, n * K);
                    for (int i = old; i < n; i++) { p0[i] = -1; p1[i] = -1; }
                    for (int i = old * K; i < n * K; i++) sg[i] = 2;
                }
                if (v < nv0)
                {
                    var P = m.Pos;
                    double x = P[3 * v], y = P[3 * v + 1], z = P[3 * v + 2];
                    for (int k = 0, i = s * K; k < K; k++, i++)
                    {
                        double d = pn[3 * k] * x + pn[3 * k + 1] * y + pn[3 * k + 2] * z - pw[k];
                        sg[i] = (sbyte)(d > eps ? 1 : d < -eps ? -1 : 0);
                    }
                }
                return s;
            }

            double Dist(int v, int k)
            {
                var P = m.Pos;
                return pn[3 * k] * P[3 * v] + pn[3 * k + 1] * P[3 * v + 1] + pn[3 * k + 2] * P[3 * v + 2] - pw[k];
            }

            int Sgn(int v, int k)
            {
                int s = SlotOf(v), i = s * K + k;
                int r = sg[i];
                if (r != 2) return r;
                double d = Dist(v, k);
                r = d > eps ? 1 : d < -eps ? -1 : 0;
                int a = p0[s];
                if (a >= 0)
                {
                    // Punkt auf Strecke (a,b): Vorzeichen verträglich mit den Endpunkten halten
                    int b = p1[s], sa = Sgn(a, k), sb = Sgn(b, k);
                    if (sa == 0 && sb == 0) r = 0;
                    else if (sa <= 0 && sb <= 0) { if (r > 0) r = 0; }
                    else if (sa >= 0 && sb >= 0) { if (r < 0) r = 0; }
                }
                sg[i] = (sbyte)r;
                return r;
            }

            // Ebenen, auf denen p liegt
            List<int> Zero(int p)
            {
                if (zs.TryGetValue(p, out var z)) return z;
                z = new List<int>();
                for (int k = 0; k < K; k++) if (Sgn(p, k) == 0) z.Add(k);
                zs[p] = z;
                return z;
            }

            static long UKey(int u, int v) => u < v ? CsgMesh.EKey(u, v) : CsgMesh.EKey(v, u);

            int CrossPt(int u, int v, int k)
            {
                int lo = Math.Min(u, v), hi = Math.Max(u, v);
                long key = CsgMesh.EKey(lo, hi);
                var M = cross[k];
                if (M.TryGetValue(key, out int x)) return x;
                double dl = Dist(lo, k), dh = Dist(hi, k), t = dl / (dl - dh);
                var P = m.Pos;
                double X = P[3 * lo] + t * (P[3 * hi] - P[3 * lo]), Y = P[3 * lo + 1] + t * (P[3 * hi + 1] - P[3 * lo + 1]), Z = P[3 * lo + 2] + t * (P[3 * hi + 2] - P[3 * lo + 2]);
                x = m.AddVertex(X, Y, Z);
                int s = SlotOf(x);
                p0[s] = lo; p1[s] = hi; sg[s * K + k] = 0;
                M[key] = x;
                for (int j = 0; j < 3; j++)
                {
                    int p = curT[j], q = curT[(j + 1) % 3];
                    long ek = UKey(p, q);
                    if (OnE(u, p, q, ek) && OnE(v, p, q, ek)) { onEdge[x] = ek; break; }
                }
                return x;
            }

            bool OnE(int x, int p, int q, long ek) => x == p || x == q || (onEdge.TryGetValue(x, out long e) && e == ek);

            List<int> ClipTri(int t)
            {
                var T = m.Tri;
                int a = T[3 * t], b = T[3 * t + 1], c = T[3 * t + 2];
                int ia = SlotOf(a) * K, ib = SlotOf(b) * K, ic = SlotOf(c) * K;
                // ganz außerhalb einer Ebene → unberührt; nirgends außerhalb → ganz innen
                bool any = false;
                for (int k = 0; k < K; k++)
                {
                    int x = sg[ia + k], y = sg[ib + k], z = sg[ic + k];
                    if (x > 0 && y > 0 && z > 0) return null;
                    if (x > 0 || y > 0 || z > 0) any = true;
                }
                if (!any) return new List<int> { a, b, c };
                curT[0] = a; curT[1] = b; curT[2] = c;
                var poly = new List<int> { a, b, c };
                var output = new List<int>();
                for (int k = 0; k < K; k++)
                {
                    bool need = false;
                    foreach (int p in poly) if (Sgn(p, k) > 0) { need = true; break; }
                    if (!need) continue;
                    output.Clear();
                    int n = poly.Count;
                    for (int i = 0; i < n; i++)
                    {
                        int u = poly[i], v = poly[(i + 1) % n], su = Sgn(u, k), sv = Sgn(v, k);
                        if (su <= 0) output.Add(u);
                        if ((su < 0 && sv > 0) || (su > 0 && sv < 0)) output.Add(CrossPt(u, v, k));
                    }
                    (poly, output) = (output, poly);
                    if (poly.Count == 0) return null;
                }
                return poly;
            }

            int Resolve(int x) { while (alias.TryGetValue(x, out int y)) x = y; return x; }

            // Körperecke zu einer Ebenenmenge (≥3 Ebenen)
            int VertexOf(List<int> Z)
            {
                foreach (int k in Z)
                    foreach (int V in C.Faces[k].Loop)
                    {
                        int c = 0;
                        foreach (int j in C.VertFaces[V]) if (Z.Contains(j)) c++;
                        if (c >= 3) return V;
                    }
                return -1;
            }

            int CvId(int V)
            {
                int r = rep[V];
                if (r >= 0) return Resolve(r);
                var p = C.Verts[V];
                // schon ein Punkt an dieser Stelle?
                int q = NearPoint(p[0], p[1], p[2]);
                if (q >= 0)
                {
                    var z = new List<int>(Zero(q));
                    foreach (int k in C.VertFaces[V]) if (!z.Contains(k)) z.Add(k);
                    zs[q] = z;
                    rep[V] = q; repOf[q] = V;
                    return q;
                }
                r = m.AddVertex(p[0], p[1], p[2]);
                int s = SlotOf(r);
                foreach (int k in C.VertFaces[V]) sg[s * K + k] = 0;
                rep[V] = r; repOf[r] = V;
                HashInsert(r);
                return r;
            }

            // Punkt p auf q legen (q bleibt), Ebenenmengen vereinigen
            void Merge(int p, int q)
            {
                var zq = new List<int>(Zero(q));
                foreach (int k in Zero(p)) if (!zq.Contains(k)) zq.Add(k);
                zs[q] = zq;
                alias[p] = q;
            }

            // Zellschlüssel (Kollisionen harmlos: es wird immer der echte Abstand geprüft)
            static int Hk(int x, int y, int z) => unchecked(x * 73856093 ^ y * 19349663 ^ z * 83492791);
            static int Fl(double x) => unchecked((int)(long)Math.Floor(x));

            void HashInsert(int p)
            {
                double s = 0.5 / eps;
                var P = m.Pos;
                int k = Hk(Fl(P[3 * p] * s), Fl(P[3 * p + 1] * s), Fl(P[3 * p + 2] * s));
                if (!hash.TryGetValue(k, out var L)) hash[k] = L = new List<int>(2);
                L.Add(p);
            }

            // ein schon eingetragener Punkt näher als eps an (x,y,z), sonst -1
            int NearPoint(double x, double y, double z)
            {
                double s = 0.5 / eps, e2 = eps * eps;
                int ix = Fl(x * s), iy = Fl(y * s), iz = Fl(z * s);
                var P = m.Pos;
                for (int a = -1; a <= 1; a++) for (int b = -1; b <= 1; b++) for (int c = -1; c <= 1; c++)
                {
                    if (!hash.TryGetValue(Hk(ix + a, iy + b, iz + c), out var L)) continue;
                    foreach (int q in L)
                    {
                        double dx = P[3 * q] - x, dy = P[3 * q + 1] - y, dz = P[3 * q + 2] - z;
                        if (dx * dx + dy * dy + dz * dz < e2) return q;
                    }
                }
                return -1;
            }

            /// <summary>Ein Versuch; null bei Erfolg (Netz dann geändert), sonst der Grund (Netz unverändert).</summary>
            public string Run()
            {
                var mn = new double[3]; var mx = new double[3];
                for (int d = 0; d < 3; d++) { mn[d] = C.Min[d] - 4 * eps; mx[d] = C.Max[d] + 4 * eps; }
                var cand = m.QueryBox(mn, mx, new List<int>());
                var touched = new List<int>();
                var polys = new List<List<int>>();
                foreach (int t in cand)
                {
                    if (m.TriOutsideBox(t, mn, mx)) continue;
                    var poly = ClipTri(t);
                    if (poly != null) { touched.Add(t); polys.Add(poly); }
                }
                var touchedSet = new HashSet<int>(touched);

                // 2a. Punkte näher als eps zusammenlegen (eine Kante unter eps lässt sich nicht gültig triangulieren);
                //     ältere Punkte (kleinere IDs, Netzecken zuerst) bleiben, Ebenenmengen werden vereinigt
                alias = new Dictionary<int, int>();
                hash = new Dictionary<int, List<int>>();
                rep = new int[C.Verts.Length]; Fill(rep, -1);
                repOf = new Dictionary<int, int>();
                var pts = new List<int>(); var seenP = new HashSet<int>();
                foreach (var poly in polys) foreach (int p in poly) if (seenP.Add(p)) pts.Add(p);
                pts.Sort();
                foreach (int p in pts)
                {
                    var P = m.Pos;
                    int q = NearPoint(P[3 * p], P[3 * p + 1], P[3 * p + 2]);
                    if (q >= 0 && !(p < nv0 && q < nv0)) { Merge(p, q); continue; }
                    HashInsert(p);
                }
                // 2b. Punkte auf ≥3 Ebenen sind Körperecken
                foreach (int pp in pts)
                {
                    int p = Resolve(pp);
                    var Z = Zero(p);
                    if (Z.Count < 3) continue;
                    int V = VertexOf(Z);
                    if (V < 0) return "Punkt auf drei Ebenen ohne Körperecke";
                    int q = rep[V] >= 0 ? Resolve(rep[V]) : -1;
                    if (q < 0) { rep[V] = p; repOf[p] = V; continue; }
                    if (q == p) continue;
                    if (p < nv0 && q < nv0) return "zwei Netzecken auf einer Körperecke";
                    if (p < q) { Merge(q, p); rep[V] = p; repOf[p] = V; } else Merge(p, q);
                }

                // 3. R_T je Dreieck
                var P3 = m.Pos; var T3 = m.Tri;
                var rAll = new Dictionary<long, int>(KeyCmp.I);      // gerichtete Kante -> Dreieck
                var rOrder = new List<long>();
                var patchEdges = new Dictionary<int, List<int>>();
                var patchOrder = new List<int>();
                var ne = new List<int>();
                var onPts = new List<int>();
                var tv = new int[3];
                for (int i = 0; i < touched.Count; i++)
                {
                    int t = touched[i];
                    var poly = polys[i];
                    tv[0] = T3[3 * t]; tv[1] = T3[3 * t + 1]; tv[2] = T3[3 * t + 2];
                    // gerichtete Kanten mit Aufhebung von Gegenkanten (wenige je Dreieck → lineare Suche)
                    ne.Clear();
                    void Add(int u, int v)
                    {
                        u = Resolve(u); v = Resolve(v);
                        if (u == v) return;
                        for (int j = 0; j < ne.Count; j += 2) if (ne[j] == v && ne[j + 1] == u) { ne.RemoveRange(j, 2); return; }
                        ne.Add(u); ne.Add(v);
                    }
                    for (int j = 0; j < 3; j++)
                    {
                        int p = tv[j], q = tv[(j + 1) % 3];
                        long ek = UKey(p, q);
                        onPts.Clear();
                        foreach (int x in poly) if (x != p && x != q && onEdge.TryGetValue(x, out long e) && e == ek) onPts.Add(x);
                        if (onPts.Count > 1)
                        {
                            double dx = P3[3 * q] - P3[3 * p], dy = P3[3 * q + 1] - P3[3 * p + 1], dz = P3[3 * q + 2] - P3[3 * p + 2];
                            double Tt(int x) => (P3[3 * x] - P3[3 * p]) * dx + (P3[3 * x + 1] - P3[3 * p + 1]) * dy + (P3[3 * x + 2] - P3[3 * p + 2]) * dz;
                            onPts.Sort((a, b) => Tt(a).CompareTo(Tt(b)));
                        }
                        int prev = p;
                        foreach (int x in onPts) { Add(prev, x); prev = x; }
                        Add(prev, q);
                    }
                    for (int j = 0; j < poly.Count; j++) Add(poly[(j + 1) % poly.Count], poly[j]);
                    int pid = m.TPatch[t];
                    if (!patchEdges.TryGetValue(pid, out var pe)) { patchEdges[pid] = pe = new List<int>(); patchOrder.Add(pid); }
                    for (int j = 0; j < ne.Count; j += 2)
                    {
                        int u = ne[j], v = ne[j + 1];
                        long key = CsgMesh.EKey(u, v);
                        for (int i2 = j + 2; i2 < ne.Count; i2 += 2) if (ne[i2] == u && ne[i2 + 1] == v) return "mehrfache Kante in R_T";
                        if (rAll.ContainsKey(key)) return "Kante in zwei Dreiecken";
                        rAll[key] = t; rOrder.Add(key);
                        pe.Add(u); pe.Add(v);
                    }
                }

                // 4. Randkanten auf ∂C → Deckel
                var capE = new List<int>[K];
                for (int k = 0; k < K; k++) capE[k] = new List<int>();
                foreach (long key in rOrder)
                {
                    int t = rAll[key], u = CsgMesh.EFrom(key), v = CsgMesh.ETo(key);
                    if (rAll.ContainsKey(CsgMesh.EKey(v, u))) continue;
                    if (m.Edges.TryGetValue(CsgMesh.EKey(v, u), out int tw) && !touchedSet.Contains(tw))
                    {
                        if (m.Edges.TryGetValue(key, out int own) && own == t) continue;
                        return "Kante ohne Gegenstück an unberührtem Nachbarn";
                    }
                    var Zu = Zero(u); var Zv = Zero(v);
                    int k0 = -1, nS = 0, k1 = -1;
                    foreach (int k in Zu) if (Zv.Contains(k)) { if (nS == 0) k0 = k; else if (nS == 1) k1 = k; nS++; }
                    if (nS == 0) return "Randkante nicht auf dem Körper";
                    int kk = k0;
                    if (nS > 1)
                    {
                        kk = nS == 2 ? PickFace(k0, k1, u, v, m.Patches[m.TPatch[t]].N) : -1;
                        if (kk < 0) return "Kante auf Körperkante nicht zuordenbar";
                    }
                    capE[kk].Add(v); capE[kk].Add(u);
                }
                if (touched.Count == 0 && !CsgAnalyze.Inside(m, C.Center)) { Changed = false; return null; }

                var capEdges = new List<int>[K];
                string capFail = BuildCaps(capE, touched.Count == 0, capEdges);
                if (capFail != null) return capFail;

                // 5. Triangulieren (innerhalb eines Patches heben sich Gegenkanten benachbarter Dreiecke auf)
                double epsLine = eps; // gleiche Toleranz wie die Klassifikation: Höhen unter eps gelten als null
                var newTris = new List<int>();
                var tri = new List<int>();
                foreach (int p in patchOrder)
                {
                    var E = NetEdges(patchEdges[p]);
                    if (E.Count == 0) continue;
                    var pa = m.Patches[p];
                    tri.Clear();
                    string r = Triangulator.TriangulateRegion(m.Pos, E, pa.U, pa.V, tri, epsLine);
                    if (r != null) return "Fläche: " + r;
                    for (int i = 0; i < tri.Count; i += 3) { newTris.Add(tri[i]); newTris.Add(tri[i + 1]); newTris.Add(tri[i + 2]); newTris.Add(p); }
                }
                var newPatches = new List<Patch>();
                for (int k = 0; k < K; k++)
                {
                    var E = capEdges[k];
                    if (E.Count == 0) continue;
                    var f = C.Faces[k];
                    var pa = new Patch(new[] { -f.N[0], -f.N[1], -f.N[2] }, -f.W, 1, 1) { UvScale = opts.CapUvScale };
                    int pid = m.Patches.Count + newPatches.Count;
                    newPatches.Add(pa);
                    tri.Clear();
                    string r = Triangulator.TriangulateRegion(m.Pos, E, pa.U, pa.V, tri, epsLine);
                    if (r != null) return "Deckel: " + r;
                    for (int i = 0; i < tri.Count; i += 3) { newTris.Add(tri[i]); newTris.Add(tri[i + 1]); newTris.Add(tri[i + 2]); newTris.Add(pid); }
                }

                // 6. Prüfen: keine entarteten oder umgeklappten Dreiecke, jede Kante genau einmal und mit Gegenstück
                var P6 = m.Pos;
                double minH = eps * 1e-2;
                for (int i = 0; i < newTris.Count; i += 4)
                {
                    int ta = newTris[i], tb = newTris[i + 1], tc = newTris[i + 2];
                    int a = 3 * ta, b = 3 * tb, c = 3 * tc;
                    double ux = P6[b] - P6[a], uy = P6[b + 1] - P6[a + 1], uz = P6[b + 2] - P6[a + 2], vx = P6[c] - P6[a], vy = P6[c + 1] - P6[a + 1], vz = P6[c + 2] - P6[a + 2];
                    double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
                    double cr = Math.Sqrt(nx * nx + ny * ny + nz * nz);
                    double wx = P6[c] - P6[b], wy = P6[c + 1] - P6[b + 1], wz = P6[c + 2] - P6[b + 2];
                    double l = Math.Sqrt(Math.Max(ux * ux + uy * uy + uz * uz, Math.Max(vx * vx + vy * vy + vz * vz, wx * wx + wy * wy + wz * wz)));
                    if (!(cr >= minH * l) || ta == tb || tb == tc || ta == tc) return "entartetes Dreieck im Ergebnis";
                    int pid = newTris[i + 3];
                    var pnr = (pid < m.Patches.Count ? m.Patches[pid] : newPatches[pid - m.Patches.Count]).N;
                    if (nx * pnr[0] + ny * pnr[1] + nz * pnr[2] <= 0) return "umgeklapptes Dreieck im Ergebnis";
                }
                var nset = new HashSet<long>(KeyCmp.I);
                for (int i = 0; i < newTris.Count; i += 4)
                    for (int j = 0; j < 3; j++)
                    {
                        long key = CsgMesh.EKey(newTris[i + j], newTris[i + (j + 1) % 3]);
                        if (!nset.Add(key)) return "doppelte Kante im Ergebnis";
                        if (m.Edges.TryGetValue(key, out int ex) && !touchedSet.Contains(ex)) return "Kante kollidiert mit Bestand";
                    }
                foreach (long key in nset)
                {
                    long twk = CsgMesh.EKey(CsgMesh.ETo(key), CsgMesh.EFrom(key));
                    if (nset.Contains(twk)) continue;
                    if (!m.Edges.TryGetValue(twk, out int ex) || touchedSet.Contains(ex)) return "offene Kante im Ergebnis";
                }
                foreach (int t in touched)
                    for (int j = 0; j < 3; j++)
                    {
                        int u = T3[3 * t + j], v = T3[3 * t + (j + 1) % 3];
                        if (m.Edges.TryGetValue(CsgMesh.EKey(v, u), out int ex) && !touchedSet.Contains(ex) && !nset.Contains(CsgMesh.EKey(u, v))) return "Nachbar verliert Kante";
                    }

                // 7. Übernehmen
                foreach (int t in touched) m.RemoveTri(t);
                m.Patches.AddRange(newPatches);
                var vseen = new HashSet<int>();
                Verts = new List<int>();
                var added = new List<int>();
                for (int i = 0; i < newTris.Count; i += 4)
                {
                    added.Add(m.AddTri(newTris[i], newTris[i + 1], newTris[i + 2], newTris[i + 3]));
                    for (int j = 0; j < 3; j++) if (vseen.Add(newTris[i + j])) Verts.Add(newTris[i + j]);
                }
                DropSkinParts(m, added, eps);
                Changed = true; Removed = touched.Count; Added = newTris.Count / 4;
                return null;
            }

            // Kante (u,v) liegt auf zwei Körperflächen: Deckel ist die Fläche, die vom Dreieck aus ins Material zeigt
            int PickFace(int k0, int k1, int u, int v, double[] nT)
            {
                var P = m.Pos;
                double ex = P[3 * v] - P[3 * u], ey = P[3 * v + 1] - P[3 * u + 1], ez = P[3 * v + 2] - P[3 * u + 2];
                double Score(int k)
                {
                    var n = C.Faces[k].N;
                    double wx = n[1] * ez - n[2] * ey, wy = n[2] * ex - n[0] * ez, wz = n[0] * ey - n[1] * ex;
                    double l = Math.Sqrt(wx * wx + wy * wy + wz * wz);
                    if (!(l > 0)) l = 1;
                    wx /= l; wy /= l; wz /= l;
                    var c = C.FaceCenter(k);
                    if (wx * (c[0] - P[3 * u]) + wy * (c[1] - P[3 * u + 1]) + wz * (c[2] - P[3 * u + 2]) < 0) { wx = -wx; wy = -wy; wz = -wz; }
                    return wx * nT[0] + wy * nT[1] + wz * nT[2];
                }
                double s0 = Score(k0), s1 = Score(k1);
                if (Math.Abs(s0 - s1) < 1e-9) return -1;
                return s0 < s1 ? k0 : k1;
            }

            struct Ev { public int P; public double T, R; public bool S; }

            // Deckel je Körperfläche: offene Ketten entlang des Flächenrands schließen
            string BuildCaps(List<int>[] capE, bool noTouch, List<int>[] edges)
            {
                var P = m.Pos;
                for (int k = 0; k < K; k++) edges[k] = new List<int>();
                var status = new int[K]; // 0 offen, 1 Ketten, 2 innen, 3 außen
                var cover = new sbyte[K][]; // je Fläche, je Deckelrandkante: 0 nichts, 1 teilweise, 2 voll
                var cl = new int[K][]; var cnb = new int[K][];
                for (int k = 0; k < K; k++)
                {
                    var L = C.Faces[k].Loop;
                    int n = L.Length;
                    var R = new int[n]; var N = new int[n];
                    for (int i = 0; i < n; i++) R[i] = L[(n - i) % n];
                    // Deckelkante i: R[i] -> R[i+1] entspricht Originalkante R[i+1] -> R[i]
                    for (int i = 0; i < n; i++) N[i] = C.Nb[k][Array.IndexOf(L, R[(i + 1) % n])];
                    cl[k] = R; cnb[k] = N;
                    cover[k] = new sbyte[n];
                }
                var deg = new Dictionary<int, int>();
                var degOrder = new List<int>();
                var starts = new List<int>(); var ends = new List<int>();
                var ev = new List<Ev>();
                for (int k = 0; k < K; k++)
                {
                    var E = capE[k];
                    if (E.Count == 0) continue;
                    var R = cl[k];
                    int n = R.Length;
                    // Ketten über Ecken-Bilanz (Ausgänge − Eingänge); Berührpunkte (2 rein, 2 raus) sind erlaubt
                    deg.Clear(); degOrder.Clear();
                    for (int i = 0; i < E.Count; i += 2)
                    {
                        for (int s = 0; s < 2; s++)
                        {
                            int p = E[i + s];
                            if (!deg.TryGetValue(p, out int d)) degOrder.Add(p);
                            deg[p] = d + (s == 0 ? 1 : -1);
                        }
                    }
                    starts.Clear(); ends.Clear();
                    foreach (int p in degOrder)
                    {
                        int d = deg[p];
                        if (d == 1) starts.Add(p); else if (d == -1) ends.Add(p); else if (d != 0) return "Deckelrand verzweigt";
                    }
                    if (starts.Count != ends.Count) return "Deckelketten unpaarig";
                    var output = edges[k];
                    output.AddRange(E);
                    if (starts.Count > 0)
                    {
                        status[k] = 1;
                        ev.Clear();
                        foreach (int p in starts) { double t = Perim(k, p, R, cnb[k]); if (t < 0) return "Kettenanfang nicht auf dem Flächenrand"; ev.Add(new Ev { P = p, T = t, R = raw, S = true }); }
                        foreach (int p in ends) { double t = Perim(k, p, R, cnb[k]); if (t < 0) return "Kettenende nicht auf dem Flächenrand"; ev.Add(new Ev { P = p, T = t, R = raw, S = false }); }
                        ev.Sort((a, b) => a.T != b.T ? a.T.CompareTo(b.T) : a.R.CompareTo(b.R));
                        for (int i = 0; i < ev.Count; i++)
                        {
                            var b = ev[(i + 1) % ev.Count];
                            if (ev.Count > 1 && ev[i].T == b.T && ev[i].R == b.R) return "Kettenpunkte fallen zusammen";
                        }
                        for (int i = 0; i < ev.Count; i++)
                        {
                            if (ev[i].S) continue;
                            var nx = ev[(i + 1) % ev.Count];
                            if (!nx.S) return "Deckelrand: Ende folgt auf Ende";
                            double pe = ev[i].T, ps = nx.T;
                            if (ps <= pe) ps += n;
                            int prev = ev[i].P;
                            for (int j = (int)Math.Floor(pe) + 1; j < ps; j++)
                            {
                                int V = CvId(R[j % n]);
                                if (V != prev) { output.Add(prev); output.Add(V); }
                                prev = V;
                            }
                            if (prev != nx.P) { output.Add(prev); output.Add(nx.P); }
                            for (int q = (int)Math.Floor(pe); q < ps; q++)
                            {
                                double a = Math.Max(pe, q), b = Math.Min(ps, q + 1);
                                if (b <= a) continue;
                                sbyte c = (sbyte)((a == q && b == q + 1) ? 2 : 1);
                                int qi = ((q % n) + n) % n;
                                if (c > cover[k][qi]) cover[k][qi] = c;
                            }
                        }
                    }
                    else
                    {
                        // nur geschlossene Schleifen: Vorzeichen der Gesamtfläche bestimmt innen (Löcher) / außen (Inseln)
                        var f = C.Faces[k];
                        var pa = new Patch(new[] { -f.N[0], -f.N[1], -f.N[2] }, 0, 1, 1);
                        int o = E[0];
                        double area = 0;
                        for (int i = 0; i < E.Count; i += 2)
                        {
                            int u = E[i], v = E[i + 1];
                            double dux = P[3 * u] - P[3 * o], duy = P[3 * u + 1] - P[3 * o + 1], duz = P[3 * u + 2] - P[3 * o + 2];
                            double dvx = P[3 * v] - P[3 * o], dvy = P[3 * v + 1] - P[3 * o + 1], dvz = P[3 * v + 2] - P[3 * o + 2];
                            double ux = dux * pa.U[0] + duy * pa.U[1] + duz * pa.U[2], uy = dux * pa.V[0] + duy * pa.V[1] + duz * pa.V[2];
                            double vx = dvx * pa.U[0] + dvy * pa.U[1] + dvz * pa.U[2], vy = dvx * pa.V[0] + dvy * pa.V[1] + dvz * pa.V[2];
                            area += ux * vy - vx * uy;
                        }
                        status[k] = area < 0 ? 2 : 3;
                    }
                }
                // Flächen ohne Ketten: Zustand von Nachbarn übernehmen
                bool changed = true;
                for (int it = 0; changed && it < K + 2; it++)
                {
                    changed = false;
                    for (int k = 0; k < K; k++)
                    {
                        if (status[k] != 0) continue;
                        for (int i = 0; i < cl[k].Length && status[k] == 0; i++)
                        {
                            int j = cnb[k][i];
                            if (status[j] == 1)
                            {
                                int ii = Array.IndexOf(cnb[j], k);
                                int c = cover[j][ii];
                                if (c == 1) return "Nachbardeckel nur teilweise";
                                status[k] = c == 2 ? 2 : 3; changed = true;
                            }
                            else if (status[j] == 2 || status[j] == 3) { status[k] = status[j]; changed = true; }
                        }
                    }
                }
                int unknownIn = -1;
                for (int k = 0; k < K; k++)
                {
                    if (status[k] != 0) continue;
                    // Prüfpunkt knapp außerhalb der Fläche (C gilt als infinitesimal vergrößert)
                    if (unknownIn < 0)
                    {
                        var c = C.FaceCenter(k); var n = C.Faces[k].N;
                        double d = 10 * eps;
                        unknownIn = CsgAnalyze.Inside(m, noTouch ? C.Center : new[] { c[0] + n[0] * d, c[1] + n[1] * d, c[2] + n[2] * d }) ? 1 : 0;
                    }
                    status[k] = unknownIn == 1 ? 2 : 3;
                }
                for (int k = 0; k < K; k++)
                {
                    if (status[k] != 2) continue;
                    // ganze Fläche (Löcher aus geschlossenen Schleifen sind schon in edges[k])
                    var R = cl[k];
                    int n = R.Length;
                    for (int i = 0; i < n; i++) { edges[k].Add(CvId(R[i])); edges[k].Add(CvId(R[(i + 1) % n])); }
                }
                return null;
            }

            // Lage eines Punkts auf dem Deckelrand von Fläche k: Kantenindex + Anteil, Ecken ganzzahlig
            double Perim(int k, int p, int[] R, int[] N)
            {
                int n = R.Length;
                raw = 0;
                if (repOf.TryGetValue(p, out int V)) { int ix = Array.IndexOf(R, V); return ix < 0 ? -1 : ix; }
                int other = -1, cnt = 0;
                foreach (int j in Zero(p)) if (j != k) { other = j; cnt++; }
                if (cnt != 1) return -1;
                int i = Array.IndexOf(N, other);
                if (i < 0) return -1;
                double[] A = C.Verts[R[i]], B = C.Verts[R[(i + 1) % n]];
                var P = m.Pos;
                double dx = B[0] - A[0], dy = B[1] - A[1], dz = B[2] - A[2], l2 = dx * dx + dy * dy + dz * dz;
                double t = ((P[3 * p] - A[0]) * dx + (P[3 * p + 1] - A[1]) * dy + (P[3 * p + 2] - A[2]) * dz) / l2;
                raw = t;
                t = Math.Min(1 - 1e-9, Math.Max(1e-9, t));
                return i + t;
            }
        }

        // Gegenkanten (u→v und v→u) heben sich auf; übrig bleibt der Rand des Bereichs
        static List<int> NetEdges(List<int> E)
        {
            var cnt = new Dictionary<long, int>(KeyCmp.I);
            var order = new List<long>();
            for (int i = 0; i < E.Count; i += 2)
            {
                int u = E[i], v = E[i + 1];
                long k = CsgMesh.EKey(u, v), r = CsgMesh.EKey(v, u);
                if (cnt.TryGetValue(r, out int c) && c > 0) { cnt[r] = c - 1; continue; }
                cnt.TryGetValue(k, out int ck);
                if (ck == 0) order.Add(k);
                cnt[k] = ck + 1;
            }
            var output = new List<int>();
            var done = new HashSet<long>(KeyCmp.I);
            foreach (long k in order)
            {
                if (!done.Add(k)) continue;
                int c = cnt[k];
                for (int j = 0; j < c; j++) { output.Add(CsgMesh.EFrom(k)); output.Add(CsgMesh.ETo(k)); }
            }
            return output;
        }

        // Teile des Netzes ohne Volumen, die neue Dreiecke enthalten, entfernen
        // (Suche je Teil begrenzt: große Teile sind keine Häute)
        static void DropSkinParts(CsgMesh m, List<int> start, double eps)
        {
            var owner = new Dictionary<int, int>(); // Dreieck → Nummer der Suche, die es gefunden hat
            var T = m.Tri;
            int search = 0;
            var L = new List<int>(); var st = new Stack<int>();
            foreach (int s in start)
            {
                if (owner.ContainsKey(s) || T[3 * s] < 0) continue;
                int id = ++search;
                L.Clear(); st.Clear();
                L.Add(s); st.Push(s);
                owner[s] = id;
                bool big = false;
                while (st.Count > 0 && !big)
                {
                    int t = st.Pop();
                    for (int j = 0; j < 3; j++)
                    {
                        if (!m.Edges.TryGetValue(CsgMesh.EKey(T[3 * t + (j + 1) % 3], T[3 * t + j]), out int o)) continue;
                        if (owner.TryGetValue(o, out int w))
                        {
                            if (w == id) continue;
                            // gehört zu einer früheren, abgebrochenen Suche → derselbe große Teil
                            big = true; break;
                        }
                        if (L.Count >= 4000) { big = true; break; }
                        owner[o] = id; L.Add(o); st.Push(o);
                    }
                }
                if (big) { foreach (int t in L) owner[t] = -1; continue; }
                VolArea(m.Pos, T, L, out double V, out double A);
                if (Math.Abs(V) < 0.5 * eps * A) foreach (int t in L) m.RemoveTri(t);
            }
        }

        static void VolArea(double[] P, int[] T, List<int> L, out double V, out double A)
        {
            int a0 = 3 * T[3 * L[0]];
            double ox = P[a0], oy = P[a0 + 1], oz = P[a0 + 2];
            V = 0; A = 0;
            foreach (int t in L)
            {
                int a = 3 * T[3 * t], b = 3 * T[3 * t + 1], c = 3 * T[3 * t + 2];
                double ax = P[a] - ox, ay = P[a + 1] - oy, az = P[a + 2] - oz, bx = P[b] - ox, by = P[b + 1] - oy, bz = P[b + 2] - oz, cx = P[c] - ox, cy = P[c + 1] - oy, cz = P[c + 2] - oz;
                V += ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx);
                double ux = bx - ax, uy = by - ay, uz = bz - az, vx = cx - ax, vy = cy - ay, vz = cz - az;
                double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
                A += Math.Sqrt(nx * nx + ny * ny + nz * nz) / 2;
            }
            V /= 6;
        }
    }
}
