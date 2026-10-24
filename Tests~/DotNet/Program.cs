// Tests of the pure C# core outside Unity.
//   dotnet run -c Release -- smoke | stress [N] [seed] [json] | bench | compare
// stress writes the cutters actually used per scenario as JSON; tools/manifold-ref.mjs replays them with
// Manifold and compares the volume (independent reference).
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using CsgForUnity;

static class Program
{
    static int fails, passes;

    static bool Check(string name, bool cond, string info = "")
    {
        if (cond) passes++;
        else { fails++; Console.WriteLine("  FAIL " + name + (info.Length > 0 ? ": " + info : "")); }
        return cond;
    }

    static int Main(string[] args)
    {
        CultureInfo.DefaultThreadCurrentCulture = CultureInfo.InvariantCulture;
        CultureInfo.CurrentCulture = CultureInfo.InvariantCulture;
        string mode = args.Length > 0 ? args[0] : "smoke";
        if (mode == "smoke") Smoke();
        else if (mode == "stress") Stress(args.Length > 1 ? int.Parse(args[1]) : 300, args.Length > 2 ? int.Parse(args[2]) : 0, args.Length > 3 ? args[3] : null);
        else if (mode == "bench") Bench();
        else if (mode == "compare") Console.Write(Compare.Run());
        Console.WriteLine($"\n{passes} ok, {fails} failed");
        return fails == 0 ? 0 : 1;
    }

    // ---------- single cases with exactly known results ----------

    static void Smoke()
    {
        Console.WriteLine("== smoke");
        // 1. box minus box, cutter faces coplanar with the mesh face → exactly 0.5 less
        {
            var m = CsgShapes.Box(1, 1, 1, 2).ToCsg();
            var r = Csg.Subtract(m, ConvexCutter.Box(new[] { 1.0, 0, 0 }, new[] { 0.5, 0.5, 0.5 }));
            Expect("coplanar notch", m, r, 8 - 0.5);
        }
        // 2. cutter fully inside → cavity, two shells
        {
            var m = CsgShapes.Box(1, 1, 1).ToCsg();
            var r = Csg.Subtract(m, ConvexCutter.Box(new[] { 0.1, 0.2, -0.1 }, new[] { 0.5, 0.5, 0.5 }));
            Expect("cavity", m, r, 7);
            Check("cavity has two shells", CsgAnalyze.Components(m).Count == 2);
        }
        // 3. cutter fully outside → nothing changes
        {
            var m = CsgShapes.Box(1, 1, 1).ToCsg();
            var r = Csg.Subtract(m, ConvexCutter.Box(new[] { 3.0, 0, 0 }, new[] { 0.5, 0.5, 0.5 }));
            Check("outside: ok, unchanged", r.Ok && !r.Changed && m.NTri == 12, r.ToString());
        }
        // 4. cutter encloses the mesh → empty
        {
            var m = CsgShapes.Box(1, 1, 1).ToCsg();
            var r = Csg.Subtract(m, ConvexCutter.Box(new[] { 0.0, 0, 0 }, new[] { 2.0, 2, 2 }));
            Check("enclosed: empty", r.Ok && m.NTri == 0, r + $" triangles {m.NTri}");
        }
        // 5. halving
        {
            var m = CsgShapes.Box(1, 1, 1, 3).ToCsg();
            var r = Csg.Subtract(m, ConvexCutter.Box(new[] { 0.0, 1.5, 0 }, new[] { 2.0, 1.5, 2 }));
            Expect("halved", m, r, 4);
        }
        // 6. through hole (prism): V = 8 − base area · 2
        {
            var m = CsgShapes.Box(1, 1, 1).ToCsg();
            var C = ConvexCutter.Cylinder(new[] { 0.0, 0, 0 }, 0.5, 2, 12);
            var r = Csg.Subtract(m, C);
            double area = 0.5 * 12 * 0.25 * Math.Sin(2 * Math.PI / 12);
            Expect("through hole", m, r, 8 - 2 * area);
            Check("through hole has genus 1 (χ = 0)", Euler(m) == 0, $"χ={Euler(m)}");
        }
        // 7. cut faces have material 1 and the right area (cutting out a corner: three quarter squares)
        {
            var m = CsgShapes.Box(1, 1, 1).ToCsg();
            var r = Csg.Subtract(m, ConvexCutter.Box(new[] { 1.0, 1, 1 }, new[] { 0.5, 0.5, 0.5 }));
            Expect("corner", m, r, 8 - 0.125);
            Check("cap area material 1", Math.Abs(MatArea(m, 1) - 0.75) < 1e-9, $"{MatArea(m, 1)}");
            Check("outer area material 0", Math.Abs(MatArea(m, 0) - (24 - 0.75)) < 1e-9, $"{MatArea(m, 0)}");
        }
        // 8. the same cut twice → the second one leaves the volume unchanged
        {
            var m = CsgShapes.Sphere(1, 3).ToCsg();
            var C = ConvexCutter.Sphere(new[] { 0.9, 0.1, 0 }, 0.4, 1, Rot.Axis(new[] { 0.0, 0, 1 }, 0.3));
            Csg.Subtract(m, C);
            double v1 = CsgAnalyze.Volume(m);
            var r = Csg.Subtract(m, C);
            Expect("repeated cut", m, r, v1);
        }
        // 9. cutter vertex exactly on a mesh vertex, cutter edge on a mesh edge
        {
            var m = CsgShapes.Box(1, 1, 1, 4).ToCsg();
            var r = Csg.Subtract(m, ConvexCutter.Box(new[] { 0.75, 0.75, 0.5 }, new[] { 0.25, 0.25, 0.25 }));
            Expect("vertex on vertex (notch)", m, r, 8 - 0.125);
            r = Csg.Subtract(m, ConvexCutter.Box(new[] { 1.0, -1, 1 }, new[] { 0.25, 0.25, 0.25 }));
            Expect("vertex on vertex (mesh corner)", m, r, 8 - 0.125 - 0.125 / 8);
        }
        // 10. attributes: outer normals stay those of the source, cut faces point against the cutter
        {
            var m = CsgShapes.Sphere(1, 2).ToCsg();
            Csg.Subtract(m, ConvexCutter.Box(new[] { 1.0, 0, 0 }, new[] { 0.5, 2, 2 }));
            var o = new double[5];
            bool okN = true;
            foreach (int t in Alive(m))
            {
                var pa = m.Patches[m.TPatch[t]];
                int a = m.Tri[3 * t];
                pa.Attr(m.Pos[3 * a], m.Pos[3 * a + 1], m.Pos[3 * a + 2], o);
                if (pa.Mat == 1 && !(o[0] > 0.999)) okN = false;
                if (pa.Mat == 0 && !(o[0] * m.Pos[3 * a] + o[1] * m.Pos[3 * a + 1] + o[2] * m.Pos[3 * a + 2] > 0.9)) okN = false;
            }
            Check("normals from patches", okN);
        }
    }

    static void Expect(string name, CsgMesh m, CutResult r, double vol)
    {
        var v = CsgAnalyze.Validate(m);
        double V = CsgAnalyze.Volume(m);
        Check(name + ": ok", r.Ok, r.ToString());
        Check(name + ": closed", v.Ok, v.ToString());
        Check(name + ": volume", Math.Abs(V - vol) < 1e-9 * Math.Max(1, Math.Abs(vol)), $"{V:R} instead of {vol:R}");
        Console.WriteLine($"  {name}: {r}, V={V:0.######}");
    }

    static IEnumerable<int> Alive(CsgMesh m) { for (int t = 0; t < m.NSlots; t++) if (m.Tri[3 * t] >= 0) yield return t; }

    static double MatArea(CsgMesh m, int mat)
    {
        double A = 0;
        var P = m.Pos;
        foreach (int t in Alive(m))
        {
            if (m.Patches[m.TPatch[t]].Mat != mat) continue;
            int a = 3 * m.Tri[3 * t], b = 3 * m.Tri[3 * t + 1], c = 3 * m.Tri[3 * t + 2];
            double ux = P[b] - P[a], uy = P[b + 1] - P[a + 1], uz = P[b + 2] - P[a + 2], vx = P[c] - P[a], vy = P[c + 1] - P[a + 1], vz = P[c + 2] - P[a + 2];
            A += Math.Sqrt(Sq(uy * vz - uz * vy) + Sq(uz * vx - ux * vz) + Sq(ux * vy - uy * vx)) / 2;
        }
        return A;
    }

    static int Euler(CsgMesh m)
    {
        var V = new HashSet<int>();
        int F = 0;
        foreach (int t in Alive(m)) { F++; for (int j = 0; j < 3; j++) V.Add(m.Tri[3 * t + j]); }
        return V.Count - 3 * F / 2 + F;
    }

    static double Sq(double x) => x * x;

    // ---------- stress: long cut sequences ----------

    static double[] RandRot(Mulberry rnd)
    {
        double x = rnd.Next() - 0.5, y = rnd.Next() - 0.5, z = rnd.Next() - 0.5, l = Math.Sqrt(x * x + y * y + z * z);
        return Rot.Axis(new[] { x / l, y / l, z / l }, rnd.Next() * Math.PI * 2);
    }

    static double[] RandDir(Mulberry rnd, double s)
    {
        double x = rnd.Next() - 0.5, y = rnd.Next() - 0.5, z = rnd.Next() - 0.5, l = Math.Sqrt(x * x + y * y + z * z);
        return new[] { x / l * s, y / l * s, z / l * s };
    }

    // as in the JS test: d.map(x => x / l * (a + rnd() * b)) – the length is re-rolled per component
    static double[] RandDirJs(Mulberry rnd, double a, double b)
    {
        double x = rnd.Next() - 0.5, y = rnd.Next() - 0.5, z = rnd.Next() - 0.5, l = Math.Sqrt(x * x + y * y + z * z);
        double px = x / l * (a + rnd.Next() * b), py = y / l * (a + rnd.Next() * b), pz = z / l * (a + rnd.Next() * b);
        return new[] { px, py, pz };
    }

    static StringBuilder json;

    // key: length of the original (German) scenario name of the JS prototype; keeps the random sequences identical to it
    static void Scenario(string name, int key, int seed, IndexedMesh src, Func<Mulberry, int, ConvexCutter> make, int n)
    {
        var m = src.ToCsg();
        var rnd = new Mulberry(key * 977 + 13 + seed * 7919);
        int retries = 0, failed = 0, done = 0, collapses = 0;
        double ms = 0, worst = 0, checkMs = 0;
        var reasons = new Dictionary<string, int>();
        var used = new List<ConvexCutter>();
        bool broken = false;
        for (int i = 0; i < n; i++)
        {
            var C = make(rnd, i);
            if (C == null) continue;
            var r = Csg.Subtract(m, C);
            ms += r.Ms; worst = Math.Max(worst, r.Ms);
            foreach (var x in r.Reasons) reasons[x] = reasons.TryGetValue(x, out int c) ? c + 1 : 1;
            if (r.Retries > 0 && r.Ok) retries++;
            collapses += r.Collapses;
            if (!r.Ok) { failed++; continue; }
            used.Add(r.Cutter);
            done++;
            var swv = Stopwatch.StartNew();
            var v = CsgAnalyze.Validate(m);
            checkMs += swv.Elapsed.TotalMilliseconds;
            if (!Check($"{name} #{i} closed", v.Ok, v.ToString())) { broken = true; break; }
        }
        var vEnd = CsgAnalyze.Validate(m);
        var sws = Stopwatch.StartNew();
        int si = broken ? -1 : CsgAnalyze.SelfIntersections(m, 20);
        double siMs = sws.Elapsed.TotalMilliseconds;
        double vol = CsgAnalyze.Volume(m);
        Check($"{name} closed at the end", vEnd.Ok, vEnd.ToString());
        Check($"{name} no self-intersection", si == 0, $"{si}");
        Check($"{name} at most 2 % rejected", failed <= n / 50, $"{failed} of {n}");
        double minAng = 180; int slivers = 0;
        foreach (int t in Alive(m)) { double a = CsgAnalyze.MinAngle(m.Pos, m.Tri[3 * t], m.Tri[3 * t + 1], m.Tri[3 * t + 2]); minAng = Math.Min(minAng, a); if (a < 5) slivers++; }
        Console.WriteLine($"{name}: {done}/{n} cuts, collapses {collapses}, {ms / n:0.00} ms/cut (max {worst:0.0}), retried {retries}, rejected {failed}, " +
                          $"{m.NTri} triangles, parts {CsgAnalyze.Components(m).Count}, V {vol:0.00000}, min angle {minAng:0.00}°, slivers<5° {100.0 * slivers / Math.Max(1, m.NTri):0.0}% [checks {checkMs / 1000:0.0} s, intersections {siMs / 1000:0.0} s]");
        if (reasons.Count > 0) Console.WriteLine("   reasons: " + string.Join("; ", reasons.Select(kv => $"{kv.Key} ×{kv.Value}")));
        if (json != null)
        {
            if (json.Length > 1) json.Append(",\n");
            json.Append("{\"name\":").Append(Str(name)).Append(",\"volume\":").Append(vol.ToString("R"));
            json.Append(",\"positions\":").Append(Arr(src.Positions)).Append(",\"indices\":[").Append(string.Join(",", src.Indices)).Append(']');
            json.Append(",\"cutters\":[");
            for (int i = 0; i < used.Count; i++)
            {
                if (i > 0) json.Append(',');
                var pts = used[i].Points ?? used[i].Verts;
                json.Append('[').Append(string.Join(",", pts.Select(p => $"[{p[0]:R},{p[1]:R},{p[2]:R}]"))).Append(']');
            }
            json.Append("]}");
        }
    }

    static string Str(string s) => "\"" + s.Replace("\\", "\\\\").Replace("\"", "\\\"") + "\"";
    static string Arr(double[] a) => "[" + string.Join(",", a.Select(x => x.ToString("R"))) + "]";

    static void Stress(int N, int seed, string jsonPath)
    {
        Console.WriteLine($"== stress N={N} seed={seed}");
        if (jsonPath != null) json = new StringBuilder("[");
        // 1. random spheres on a sphere
        Scenario("Sphere × spheres", 14, seed, CsgShapes.Sphere(1, 3), (rnd, i) =>
            ConvexCutter.Sphere(RandDirJs(rnd, 0.7, 0.4), 0.15 + rnd.Next() * 0.25, rnd.Next() < 0.5 ? 0 : 1, RandRot(rnd)), N);
        // 2. axis-aligned boxes on grid positions (lots of coplanarity: equal planes, edges on edges)
        Scenario("Box × grid boxes", 21, seed, CsgShapes.Box(1, 1, 1, 2), (rnd, i) =>
        {
            double G() => Math.Round((rnd.Next() * 2.4 - 1.2) * 4) / 4;
            double H() => Math.Round((0.25 + rnd.Next() * 0.5) * 4) / 4;
            return ConvexCutter.Box(new[] { G(), G(), G() }, new[] { H(), H(), H() });
        }, N);
        // 3. the same cut repeatedly, shifted along an axis (caps on old caps)
        Scenario("Box × slider", 17, seed, CsgShapes.Box(1, 1, 1), (rnd, i) =>
            ConvexCutter.Box(new[] { -1 + (i % 20) * 0.1, 1, (i / 20 % 5) * 0.2 - 0.4 }, new[] { 0.3, 0.25, 0.2 }), 100);
        // 4. random rocks with random rotation on a torus
        Scenario("Torus × rocks", 15, seed, CsgShapes.Torus(1, 0.4, 48, 24), (rnd, i) =>
        {
            double a = rnd.Next() * Math.PI * 2, b = rnd.Next() * Math.PI * 2;
            var c = new[] { (1 + 0.4 * Math.Cos(b)) * Math.Cos(a), 0.4 * Math.Sin(b), (1 + 0.4 * Math.Cos(b)) * Math.Sin(a) };
            return ConvexCutter.Rock(c, 0.1 + rnd.Next() * 0.25, 14, i + 1, RandRot(rnd));
        }, N);
        // 5. rock (flat triangles) × cylinders in all directions
        Scenario("Rock × cylinders", 15, seed, CsgShapes.Rock(1, 3), (rnd, i) =>
            ConvexCutter.Cylinder(RandDirJs(rnd, 0.6, 0.5), 0.05 + rnd.Next() * 0.2, 0.3 + rnd.Next() * 0.6, 12, RandRot(rnd)), N);
        // 6. thin rod, cross cuts (thin walls, severing)
        Scenario("Rod × plates", 14, seed, CsgShapes.Cylinder(0.1, 2, 16, 8), (rnd, i) =>
        {
            double y = -1.9 + rnd.Next() * 3.8;
            return ConvexCutter.Box(new[] { rnd.Next() * 0.2 - 0.1, y, rnd.Next() * 0.2 - 0.1 }, new[] { 0.08 + rnd.Next() * 0.1, 0.02 + rnd.Next() * 0.05, 0.3 }, RandRot(rnd));
        }, Math.Min(N, 150));
        // 7. vertices exactly on mesh vertices, edges on edges
        Scenario("Box × exact corners", 20, seed, CsgShapes.Box(1, 1, 1, 4), (rnd, i) =>
        {
            double G() => (Math.Floor(rnd.Next() * 9) - 4) / 4;
            return ConvexCutter.Box(new[] { G(), G(), G() }, new[] { 0.25, 0.25, 0.25 });
        }, Math.Min(N, 200));
        // 8. tiny to medium cutters mixed (scale over 3.5 orders of magnitude, down to 10 eps)
        Scenario("Sphere × scales", 16, seed, CsgShapes.Sphere(1, 3), (rnd, i) =>
        {
            double s = Math.Pow(10, -4 + rnd.Next() * 3.5);
            return ConvexCutter.Rock(RandDir(rnd, 1 + (rnd.Next() - 0.5) * s), s, 10 + (int)(rnd.Next() * 20), i + 7, RandRot(rnd));
        }, N);
        if (json != null)
        {
            json.Append("]\n");
            File.WriteAllText(jsonPath, json.ToString());
            Console.WriteLine("cutters for the Manifold comparison: " + jsonPath);
        }
    }

    // ---------- speed ----------

    static void Bench()
    {
        Console.WriteLine("== bench (median per cut, first run excluded)");
        foreach (int detail in new[] { 4, 5, 6, 7 })
        {
            var src = CsgShapes.Sphere(1, detail);
            var sw = Stopwatch.StartNew();
            var m = src.ToCsg();
            double imp = sw.Elapsed.TotalMilliseconds;
            var rnd = new Mulberry(42);
            var times = new List<double>();
            for (int i = 0; i < 40; i++)
            {
                var C = ConvexCutter.Sphere(RandDir(rnd, 1), 0.15, 1, RandRot(rnd));
                var r = Csg.Subtract(m, C);
                if (i > 0) times.Add(r.Ms);
                if (!r.Ok) Console.WriteLine("  rejected: " + r.Reason);
            }
            times.Sort();
            var v = CsgAnalyze.Validate(m);
            Check($"bench {detail} closed", v.Ok, v.ToString());
            Console.WriteLine($"  sphere {src.Indices.Length / 3,7} triangles: import {imp,7:0.0} ms, cut median {times[times.Count / 2]:0.00} ms, max {times[times.Count - 1]:0.00} ms");
        }
    }
}
