// Speed comparison with the JS prototype and Manifold: same meshes, same cutter sequence (mulberry32) and the same
// median as 27-convex-csg/tools/bench.mjs, so the tables can be compared line by line.
// The body between the markers also runs unchanged in the Unity editor (unity command eval).
static class Compare
{
    public static string Run()
    {
        // BEGIN BODY
        var sb = new System.Text.StringBuilder();
        System.Func<int, double, int, System.Collections.Generic.List<CsgForUnity.ConvexCutter>> cutters = (n, r, seed) =>
        {
            var rnd = new CsgForUnity.Mulberry(seed);
            var o = new System.Collections.Generic.List<CsgForUnity.ConvexCutter>();
            for (int i = 0; i < n; i++)
            {
                double dx = rnd.Next() - 0.5, dy = rnd.Next() - 0.5, dz = rnd.Next() - 0.5, l = System.Math.Sqrt(dx * dx + dy * dy + dz * dz);
                double ax = rnd.Next() - 0.5, ay = rnd.Next() - 0.5, az = rnd.Next() - 0.5, la = System.Math.Sqrt(ax * ax + ay * ay + az * az);
                // same evaluation order as the JS call: centre (one random per component), radius, angle
                double cx = dx / l * (0.97 + rnd.Next() * 0.06), cy = dy / l * (0.97 + rnd.Next() * 0.06), cz = dz / l * (0.97 + rnd.Next() * 0.06);
                double rr = r * (0.8 + 0.4 * rnd.Next());
                var R = CsgForUnity.Rot.Axis(new[] { ax / la, ay / la, az / la }, rnd.Next() * 6);
                o.Add(CsgForUnity.ConvexCutter.Sphere(new[] { cx, cy, cz }, rr, 1, R));
            }
            return o;
        };
        System.Func<int, double, int, int, string> run = (detail, r, n, seed) =>
        {
            var src = CsgForUnity.CsgShapes.Sphere(1, detail);
            var cs = cutters(n, r, seed);
            var m = src.ToCsg();
            var t = new System.Collections.Generic.List<double>();
            int rejected = 0;
            foreach (var c in cs)
            {
                var res = CsgForUnity.Csg.Subtract(m, c);
                t.Add(res.Ms);
                if (!res.Ok) rejected++;
            }
            t.Sort();
            return string.Format(System.Globalization.CultureInfo.InvariantCulture, "{0,9}{1,9:0.00}{2,11}{3,12:0.000000}{4,6}",
                src.Indices.Length / 3, t[t.Count / 2], m.NTri, CsgForUnity.CsgAnalyze.Volume(m), rejected);
        };
        sb.Append("1) mesh size grows, small sphere cutter (r~0.08, 80 planes), 40 cuts, median ms/cut\n");
        sb.Append("     tris  median  tris after      volume  rej\n");
        foreach (int d in new[] { 3, 4, 5, 6, 7 }) sb.Append(run(d, 0.08, 40, 7)).Append('\n');
        sb.Append("2) cutter size grows, mesh 82k tris, 20 cuts\n");
        foreach (double r in new[] { 0.03, 0.1, 0.25, 0.5, 0.8 }) sb.Append($"r={r,-5}").Append(run(6, r, 20, 11)).Append('\n');
        return sb.ToString();
        // END BODY
    }
}
