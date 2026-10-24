using System;
using System.Collections.Generic;

namespace CsgForUnity
{
    public struct ValidationResult
    {
        public bool Ok;
        public int Triangles, Open, Duplicate, Degenerate;
        public override string ToString() => Ok ? $"geschlossen, manifold ({Triangles} Dreiecke)" : $"offen {Open}, doppelt {Duplicate}, entartet {Degenerate}";
    }

    /// <summary>Prüfungen und Kennzahlen: geschlossen/manifold, Volumen, Fläche, Zusammenhang, Punkt im Netz, Dreieckschnitte.</summary>
    public static class CsgAnalyze
    {
        /// <summary>Jede gerichtete Kante genau einmal und mit Gegenkante; keine entarteten Dreiecke.</summary>
        public static ValidationResult Validate(CsgMesh m)
        {
            var seen = new HashSet<long>(KeyCmp.I);
            var r = new ValidationResult();
            double minH = m.Eps * 1e-2;
            var P = m.Pos; var T = m.Tri;
            for (int t = 0; t < m.NSlots; t++)
            {
                if (T[3 * t] < 0) continue;
                r.Triangles++;
                int a = T[3 * t], b = T[3 * t + 1], c = T[3 * t + 2];
                if (!seen.Add(CsgMesh.EKey(a, b))) r.Duplicate++;
                if (!seen.Add(CsgMesh.EKey(b, c))) r.Duplicate++;
                if (!seen.Add(CsgMesh.EKey(c, a))) r.Duplicate++;
                double e1x = P[3 * b] - P[3 * a], e1y = P[3 * b + 1] - P[3 * a + 1], e1z = P[3 * b + 2] - P[3 * a + 2];
                double e2x = P[3 * c] - P[3 * a], e2y = P[3 * c + 1] - P[3 * a + 1], e2z = P[3 * c + 2] - P[3 * a + 2];
                double cx = e1y * e2z - e1z * e2y, cy = e1z * e2x - e1x * e2z, cz = e1x * e2y - e1y * e2x;
                double e3x = P[3 * c] - P[3 * b], e3y = P[3 * c + 1] - P[3 * b + 1], e3z = P[3 * c + 2] - P[3 * b + 2];
                double lmax = Math.Sqrt(Math.Max(e1x * e1x + e1y * e1y + e1z * e1z, Math.Max(e2x * e2x + e2y * e2y + e2z * e2z, e3x * e3x + e3y * e3y + e3z * e3z)));
                if (Math.Sqrt(cx * cx + cy * cy + cz * cz) < minH * lmax || a == b || b == c || c == a) r.Degenerate++;
            }
            foreach (long k in seen) if (!seen.Contains(CsgMesh.EKey(CsgMesh.ETo(k), CsgMesh.EFrom(k)))) r.Open++;
            r.Ok = r.Open == 0 && r.Duplicate == 0 && r.Degenerate == 0;
            return r;
        }

        public static double Volume(CsgMesh m)
        {
            var mn = new double[3]; var mx = new double[3];
            m.Bounds(mn, mx);
            double ox = (mn[0] + mx[0]) / 2, oy = (mn[1] + mx[1]) / 2, oz = (mn[2] + mx[2]) / 2, V = 0;
            var P = m.Pos; var T = m.Tri;
            for (int t = 0; t < m.NSlots; t++)
            {
                if (T[3 * t] < 0) continue;
                int a = 3 * T[3 * t], b = 3 * T[3 * t + 1], c = 3 * T[3 * t + 2];
                double ax = P[a] - ox, ay = P[a + 1] - oy, az = P[a + 2] - oz, bx = P[b] - ox, by = P[b + 1] - oy, bz = P[b + 2] - oz, cx = P[c] - ox, cy = P[c + 1] - oy, cz = P[c + 2] - oz;
                V += ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx);
            }
            return V / 6;
        }

        public static double Area(CsgMesh m)
        {
            double A = 0;
            var P = m.Pos; var T = m.Tri;
            for (int t = 0; t < m.NSlots; t++)
            {
                if (T[3 * t] < 0) continue;
                int a = 3 * T[3 * t], b = 3 * T[3 * t + 1], c = 3 * T[3 * t + 2];
                double ux = P[b] - P[a], uy = P[b + 1] - P[a + 1], uz = P[b + 2] - P[a + 2], vx = P[c] - P[a], vy = P[c + 1] - P[a + 1], vz = P[c + 2] - P[a + 2];
                double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
                A += Math.Sqrt(nx * nx + ny * ny + nz * nz) / 2;
            }
            return A;
        }

        /// <summary>Kleinster Innenwinkel des Dreiecks in Grad.</summary>
        public static double MinAngle(double[] P, int a, int b, int c) => Math.Min(Ang(P, a, b, c), Math.Min(Ang(P, b, c, a), Ang(P, c, a, b)));

        static double Ang(double[] P, int p, int q, int r)
        {
            double ux = P[3 * q] - P[3 * p], uy = P[3 * q + 1] - P[3 * p + 1], uz = P[3 * q + 2] - P[3 * p + 2];
            double vx = P[3 * r] - P[3 * p], vy = P[3 * r + 1] - P[3 * p + 1], vz = P[3 * r + 2] - P[3 * p + 2];
            double cx = uy * vz - uz * vy, cy = uz * vx - ux * vz, cz = ux * vy - uy * vx;
            return Math.Atan2(Math.Sqrt(cx * cx + cy * cy + cz * cz), ux * vx + uy * vy + uz * vz) * 180 / Math.PI;
        }

        /// <summary>Zusammenhangskomponenten über gemeinsame Kanten → Listen von Dreiecken.</summary>
        public static List<List<int>> Components(CsgMesh m)
        {
            var comp = new int[m.NSlots];
            for (int i = 0; i < comp.Length; i++) comp[i] = -1;
            var o = new List<List<int>>();
            var T = m.Tri;
            var st = new Stack<int>();
            for (int s = 0; s < m.NSlots; s++)
            {
                if (T[3 * s] < 0 || comp[s] >= 0) continue;
                int id = o.Count;
                var L = new List<int> { s };
                comp[s] = id; st.Push(s);
                while (st.Count > 0)
                {
                    int t = st.Pop();
                    for (int j = 0; j < 3; j++)
                        if (m.Edges.TryGetValue(CsgMesh.EKey(T[3 * t + (j + 1) % 3], T[3 * t + j]), out int nb) && comp[nb] < 0) { comp[nb] = id; L.Add(nb); st.Push(nb); }
                }
                o.Add(L);
            }
            return o;
        }

        static readonly double[][] RayDirs = { new[] { 0.5377, 0.8317, 0.1379 }, new[] { -0.3112, 0.2297, 0.9221 }, new[] { 0.7071, -0.6, 0.3742 }, new[] { -0.83, -0.41, -0.38 }, new[] { 0.11, -0.97, 0.21 } };

        /// <summary>Punkt im Netz? Strahlparität über alle Dreiecke, mehrere Richtungen, Mehrheit.</summary>
        public static bool Inside(CsgMesh m, double[] p)
        {
            int votes = 0, total = 0;
            var P = m.Pos; var T = m.Tri;
            foreach (var d0 in RayDirs)
            {
                double l = Math.Sqrt(d0[0] * d0[0] + d0[1] * d0[1] + d0[2] * d0[2]);
                double dx = d0[0] / l, dy = d0[1] / l, dz = d0[2] / l;
                int hits = 0; bool bad = false;
                for (int t = 0; t < m.NSlots && !bad; t++)
                {
                    if (T[3 * t] < 0) continue;
                    int a = 3 * T[3 * t], b = 3 * T[3 * t + 1], c = 3 * T[3 * t + 2];
                    double e1x = P[b] - P[a], e1y = P[b + 1] - P[a + 1], e1z = P[b + 2] - P[a + 2], e2x = P[c] - P[a], e2y = P[c + 1] - P[a + 1], e2z = P[c + 2] - P[a + 2];
                    double hx = dy * e2z - dz * e2y, hy = dz * e2x - dx * e2z, hz = dx * e2y - dy * e2x;
                    double det = e1x * hx + e1y * hy + e1z * hz;
                    if (Math.Abs(det) < 1e-300) continue;
                    double sx = p[0] - P[a], sy = p[1] - P[a + 1], sz = p[2] - P[a + 2];
                    double u = (sx * hx + sy * hy + sz * hz) / det;
                    if (u < -1e-9 || u > 1 + 1e-9) continue;
                    double qx = sy * e1z - sz * e1y, qy = sz * e1x - sx * e1z, qz = sx * e1y - sy * e1x;
                    double v = (dx * qx + dy * qy + dz * qz) / det;
                    if (v < -1e-9 || u + v > 1 + 1e-9) continue;
                    double tt = (e2x * qx + e2y * qy + e2z * qz) / det;
                    if (tt < -1e-12) continue;
                    if (u < 1e-9 || v < 1e-9 || u + v > 1 - 1e-9 || tt < 1e-9) { bad = true; break; }
                    hits++;
                }
                if (bad) continue;
                total++; if ((hits & 1) == 1) votes++;
                if (total >= 3) break;
            }
            return total > 0 && votes * 2 > total;
        }

        // ---------- Dreieckschnitte (Aufräumen, Tests) ----------

        [ThreadStatic] static List<int> hitCand;

        /// <summary>
        /// Schneidet das Dreieck (a,b,c) ein lebendes Dreieck des Netzes, das keine Ecke mit ihm teilt?
        /// Berührungen (Abstand ≤ eps) zählen nicht. ignore: Dreiecke, die nicht zählen.
        /// </summary>
        public static bool TriHitsMesh(CsgMesh m, int a, int b, int c, HashSet<int> ignore) => Hits(m, a, b, c, ignore, -1);

        // skipUpTo: Dreiecke mit Nummer ≤ skipUpTo zählen nicht (jedes Paar nur einmal prüfen)
        static bool Hits(CsgMesh m, int a, int b, int c, HashSet<int> ignore, int skipUpTo)
        {
            var P = m.Pos; var T = m.Tri; double e = m.Eps;
            var mn = new[] { CsgMesh.Min3(P[3 * a], P[3 * b], P[3 * c]) - e, CsgMesh.Min3(P[3 * a + 1], P[3 * b + 1], P[3 * c + 1]) - e, CsgMesh.Min3(P[3 * a + 2], P[3 * b + 2], P[3 * c + 2]) - e };
            var mx = new[] { CsgMesh.Max3(P[3 * a], P[3 * b], P[3 * c]) + e, CsgMesh.Max3(P[3 * a + 1], P[3 * b + 1], P[3 * c + 1]) + e, CsgMesh.Max3(P[3 * a + 2], P[3 * b + 2], P[3 * c + 2]) + e };
            var cand = hitCand ??= new List<int>();
            cand.Clear();
            m.QueryBox(mn, mx, cand);
            foreach (int s in cand)
            {
                if (s <= skipUpTo || T[3 * s] < 0 || (ignore != null && ignore.Contains(s)) || m.TriOutsideBox(s, mn, mx)) continue;
                int d0 = T[3 * s], d1 = T[3 * s + 1], d2 = T[3 * s + 2];
                if (d0 == a || d0 == b || d0 == c || d1 == a || d1 == b || d1 == c || d2 == a || d2 == b || d2 == c) continue;
                if (SegTri(P, a, b, d0, d1, d2, e) || SegTri(P, b, c, d0, d1, d2, e) || SegTri(P, c, a, d0, d1, d2, e) ||
                    SegTri(P, d0, d1, a, b, c, e) || SegTri(P, d1, d2, a, b, c, e) || SegTri(P, d2, d0, a, b, c, e)) return true;
            }
            return false;
        }

        /// <summary>Anzahl echter Selbstdurchdringungen (Paare nicht benachbarter Dreiecke), bis maxCount.</summary>
        public static int SelfIntersections(CsgMesh m, int maxCount = int.MaxValue)
        {
            var T = m.Tri;
            int count = 0;
            for (int t = 0; t < m.NSlots && count < maxCount; t++)
            {
                if (T[3 * t] < 0) continue;
                if (Hits(m, T[3 * t], T[3 * t + 1], T[3 * t + 2], null, t)) count++;
            }
            return count;
        }

        static bool SegTri(double[] P, int p, int q, int a, int b, int c, double e)
        {
            double ax = P[3 * a], ay = P[3 * a + 1], az = P[3 * a + 2];
            double ux = P[3 * b] - ax, uy = P[3 * b + 1] - ay, uz = P[3 * b + 2] - az, vx = P[3 * c] - ax, vy = P[3 * c + 1] - ay, vz = P[3 * c + 2] - az;
            double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx, nl = Math.Sqrt(nx * nx + ny * ny + nz * nz);
            if (!(nl > 0)) return false;
            double px = P[3 * p], py = P[3 * p + 1], pz = P[3 * p + 2], qx = P[3 * q], qy = P[3 * q + 1], qz = P[3 * q + 2];
            double dp = (nx * (px - ax) + ny * (py - ay) + nz * (pz - az)) / nl;
            double dq = (nx * (qx - ax) + ny * (qy - ay) + nz * (qz - az)) / nl;
            if (!((dp > e && dq < -e) || (dp < -e && dq > e))) return false;
            double t = dp / (dp - dq);
            double X0 = px + t * (qx - px), X1 = py + t * (qy - py), X2 = pz + t * (qz - pz);
            // Abstand von X zu jeder Dreieckskante (in der Ebene) > e und auf der Innenseite
            return EdgeIn(P, a, b, X0, X1, X2, nx, ny, nz, nl, e) && EdgeIn(P, b, c, X0, X1, X2, nx, ny, nz, nl, e) && EdgeIn(P, c, a, X0, X1, X2, nx, ny, nz, nl, e);
        }

        static bool EdgeIn(double[] P, int i, int j, double X0, double X1, double X2, double nx, double ny, double nz, double nl, double e)
        {
            double ex = P[3 * j] - P[3 * i], ey = P[3 * j + 1] - P[3 * i + 1], ez = P[3 * j + 2] - P[3 * i + 2];
            double wx = X0 - P[3 * i], wy = X1 - P[3 * i + 1], wz = X2 - P[3 * i + 2];
            double cx = ey * wz - ez * wy, cy = ez * wx - ex * wz, cz = ex * wy - ey * wx;
            return (cx * nx + cy * ny + cz * nz) / nl / Math.Sqrt(ex * ex + ey * ey + ez * ez) > e;
        }
    }
}
