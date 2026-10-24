using System;
using System.Collections.Generic;

namespace CsgForUnity
{
    /// <summary>Indizierte Dreiecke (xyz, Indizes, Normalen xyz, UVs uv je Ecke).</summary>
    public sealed class IndexedMesh
    {
        public double[] Positions, Normals, Uvs;
        public int[] Indices;
        public CsgMesh ToCsg() => CsgMesh.FromIndexed(Positions, Indices, Normals, Uvs);
    }

    /// <summary>Testnetze (geschlossen, außen gegen den Uhrzeigersinn).</summary>
    public static class CsgShapes
    {
        /// <summary>Quader, jede Seite in seg×seg Quadrate unterteilt, flache Normalen, UV je Seite.</summary>
        public static IndexedMesh Box(double hx, double hy, double hz, int seg = 1)
        {
            var pos = new List<double>(); var nrm = new List<double>(); var uv = new List<double>(); var idx = new List<int>();
            int[][][] faces =
            {
                new[] { new[] { 1, 0, 0 }, new[] { 0, 0, -1 }, new[] { 0, 1, 0 } }, new[] { new[] { -1, 0, 0 }, new[] { 0, 0, 1 }, new[] { 0, 1, 0 } },
                new[] { new[] { 0, 1, 0 }, new[] { 1, 0, 0 }, new[] { 0, 0, -1 } }, new[] { new[] { 0, -1, 0 }, new[] { 1, 0, 0 }, new[] { 0, 0, 1 } },
                new[] { new[] { 0, 0, 1 }, new[] { 1, 0, 0 }, new[] { 0, 1, 0 } }, new[] { new[] { 0, 0, -1 }, new[] { -1, 0, 0 }, new[] { 0, 1, 0 } },
            };
            double[] h = { hx, hy, hz };
            foreach (var f in faces)
            {
                int[] n = f[0], u = f[1], v = f[2];
                int b0 = pos.Count / 3;
                double hu = Math.Abs(u[0] * hx + u[1] * hy + u[2] * hz), hv = Math.Abs(v[0] * hx + v[1] * hy + v[2] * hz);
                for (int j = 0; j <= seg; j++)
                    for (int i = 0; i <= seg; i++)
                    {
                        double s = (double)i / seg * 2 - 1, t = (double)j / seg * 2 - 1;
                        for (int d = 0; d < 3; d++) { pos.Add((n[d] + u[d] * s + v[d] * t) * h[d]); nrm.Add(n[d]); }
                        uv.Add((s + 1) / 2 * hu); uv.Add((t + 1) / 2 * hv);
                    }
                for (int j = 0; j < seg; j++)
                    for (int i = 0; i < seg; i++)
                    {
                        int a = b0 + j * (seg + 1) + i, b = a + 1, c = a + seg + 1, d = c + 1;
                        idx.AddRange(new[] { a, b, d, a, d, c });
                    }
            }
            return Make(pos, idx, nrm, uv);
        }

        /// <summary>Unterteiltes Ikosaeder als Kugel mit glatten Normalen.</summary>
        public static IndexedMesh Sphere(double r, int detail)
        {
            var (V, F) = ConvexCutter.IcoPoints(detail);
            var pos = new List<double>(); var nrm = new List<double>(); var uv = new List<double>(); var idx = new List<int>();
            foreach (var v in V)
            {
                pos.Add(v[0] * r); pos.Add(v[1] * r); pos.Add(v[2] * r);
                nrm.Add(v[0]); nrm.Add(v[1]); nrm.Add(v[2]);
                uv.Add(Math.Atan2(v[2], v[0]) / (2 * Math.PI) * 4 * r); uv.Add(Math.Acos(Math.Max(-1, Math.Min(1, v[1]))) / Math.PI * 2 * r);
            }
            foreach (var f in F) idx.AddRange(f);
            return Make(pos, idx, nrm, uv);
        }

        public static IndexedMesh Torus(double R, double r, int nu, int nv)
        {
            var pos = new List<double>(); var nrm = new List<double>(); var uv = new List<double>(); var idx = new List<int>();
            for (int i = 0; i < nu; i++)
                for (int j = 0; j < nv; j++)
                {
                    double a = (double)i / nu * Math.PI * 2, b = (double)j / nv * Math.PI * 2;
                    double cx = Math.Cos(a), cz = Math.Sin(a), n0 = Math.Cos(b) * cx, n1 = Math.Sin(b), n2 = Math.Cos(b) * cz;
                    pos.Add(R * cx + r * n0); pos.Add(r * n1); pos.Add(R * cz + r * n2);
                    nrm.Add(n0); nrm.Add(n1); nrm.Add(n2); uv.Add(a * R); uv.Add(b * r);
                }
            int Id(int i, int j) => (i % nu) * nv + (j % nv);
            for (int i = 0; i < nu; i++)
                for (int j = 0; j < nv; j++)
                {
                    int a = Id(i, j), b = Id(i + 1, j), c = Id(i, j + 1), d = Id(i + 1, j + 1);
                    idx.AddRange(new[] { a, c, d, a, d, b });
                }
            return Make(pos, idx, nrm, uv);
        }

        /// <summary>Verbeulte Kugel mit flachen Dreiecken (jedes Dreieck eigene Ebene).</summary>
        public static IndexedMesh Rock(double r, int detail, int seed = 3)
        {
            var (V, F) = ConvexCutter.IcoPoints(detail);
            var rnd = new Mulberry(seed);
            var waves = new double[6][];
            for (int i = 0; i < 6; i++)
            {
                double dx = rnd.Next() - 0.5, dy = rnd.Next() - 0.5, dz = rnd.Next() - 0.5, l = Math.Sqrt(dx * dx + dy * dy + dz * dz);
                waves[i] = new[] { dx / l, dy / l, dz / l, 1 + rnd.Next() * 3, rnd.Next() * 6, 0.04 + rnd.Next() * 0.06 };
            }
            var Pt = new List<double[]>();
            foreach (var v in V)
            {
                double s = 1;
                foreach (var w in waves) s += w[5] * Math.Sin(w[3] * (v[0] * w[0] + v[1] * w[1] + v[2] * w[2]) * 3 + w[4]);
                Pt.Add(new[] { v[0] * r * s, v[1] * r * s, v[2] * r * s });
            }
            var pos = new List<double>(); var nrm = new List<double>(); var uv = new List<double>(); var idx = new List<int>();
            foreach (var f in F)
            {
                double[] A = Pt[f[0]], B = Pt[f[1]], Cc = Pt[f[2]];
                double e1x = B[0] - A[0], e1y = B[1] - A[1], e1z = B[2] - A[2], e2x = Cc[0] - A[0], e2y = Cc[1] - A[1], e2z = Cc[2] - A[2];
                double nx = e1y * e2z - e1z * e2y, ny = e1z * e2x - e1x * e2z, nz = e1x * e2y - e1y * e2x, l = Math.Sqrt(nx * nx + ny * ny + nz * nz);
                foreach (int q in f)
                {
                    idx.Add(pos.Count / 3);
                    pos.AddRange(Pt[q]); nrm.Add(nx / l); nrm.Add(ny / l); nrm.Add(nz / l);
                    uv.Add(Pt[q][0] + Pt[q][2]); uv.Add(Pt[q][1]);
                }
            }
            return Make(pos, idx, nrm, uv);
        }

        /// <summary>Zylinder (Stab) entlang y mit flachen Deckeln.</summary>
        public static IndexedMesh Cylinder(double r, double hh, int segs, int rings = 1)
        {
            var pos = new List<double>(); var nrm = new List<double>(); var uv = new List<double>(); var idx = new List<int>();
            for (int j = 0; j <= rings; j++)
                for (int i = 0; i < segs; i++)
                {
                    double a = (double)i / segs * Math.PI * 2, y = -hh + 2 * hh * j / rings;
                    pos.Add(r * Math.Cos(a)); pos.Add(y); pos.Add(r * Math.Sin(a));
                    nrm.Add(Math.Cos(a)); nrm.Add(0); nrm.Add(Math.Sin(a)); uv.Add(a * r); uv.Add(y);
                }
            for (int j = 0; j < rings; j++)
                for (int i = 0; i < segs; i++)
                {
                    int a = j * segs + i, b = j * segs + (i + 1) % segs, c = a + segs, d = b + segs;
                    idx.AddRange(new[] { a, c, d, a, d, b });
                }
            foreach (var s in new[] { -1, 1 })
            {
                double y = s * hh;
                int b0 = pos.Count / 3;
                pos.Add(0); pos.Add(y); pos.Add(0); nrm.Add(0); nrm.Add(s); nrm.Add(0); uv.Add(0); uv.Add(0);
                for (int i = 0; i < segs; i++)
                {
                    double a = (double)i / segs * Math.PI * 2;
                    pos.Add(r * Math.Cos(a)); pos.Add(y); pos.Add(r * Math.Sin(a)); nrm.Add(0); nrm.Add(s); nrm.Add(0); uv.Add(r * Math.Cos(a)); uv.Add(r * Math.Sin(a));
                }
                for (int i = 0; i < segs; i++)
                {
                    int a = b0 + 1 + i, b = b0 + 1 + (i + 1) % segs;
                    if (s > 0) idx.AddRange(new[] { b0, b, a }); else idx.AddRange(new[] { b0, a, b });
                }
            }
            return Make(pos, idx, nrm, uv);
        }

        static IndexedMesh Make(List<double> pos, List<int> idx, List<double> nrm, List<double> uv) =>
            new IndexedMesh { Positions = pos.ToArray(), Indices = idx.ToArray(), Normals = nrm.ToArray(), Uvs = uv.ToArray() };
    }
}
