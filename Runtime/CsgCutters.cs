using System.Collections.Generic;
using UnityEngine;

namespace CsgForUnity
{
    /// <summary>Schneidkörper mit Unity-Typen (Weltkoordinaten), dazu ein Vorschau-Mesh.</summary>
    public static class CsgCutters
    {
        static double[] D(Vector3 v) => new double[] { v.x, v.y, v.z };

        static double[] R(Quaternion q)
        {
            var m = Matrix4x4.Rotate(q);
            return new double[] { m.m00, m.m01, m.m02, m.m10, m.m11, m.m12, m.m20, m.m21, m.m22 };
        }

        public static ConvexCutter Box(Vector3 center, Vector3 halfExtents, Quaternion rotation) =>
            ConvexCutter.Box(D(center), D(halfExtents), R(rotation));

        /// <summary>Kugel als unterteiltes Ikosaeder: detail 0 = 20 Flächen, 1 = 80, 2 = 320.</summary>
        public static ConvexCutter Sphere(Vector3 center, float radius, Quaternion rotation, int detail = 1) =>
            ConvexCutter.Sphere(D(center), radius, detail, R(rotation));

        /// <summary>Zylinder entlang der lokalen y-Achse; height ist die volle Höhe.</summary>
        public static ConvexCutter Cylinder(Vector3 center, float radius, float height, Quaternion rotation, int segments = 16) =>
            ConvexCutter.Cylinder(D(center), radius, height / 2, segments, R(rotation));

        /// <summary>Unregelmäßiger Brocken aus zufälligen Punkten (gleicher seed → gleiche Form).</summary>
        public static ConvexCutter Rock(Vector3 center, float radius, int seed, Quaternion rotation, int points = 16) =>
            ConvexCutter.Rock(D(center), radius, points, seed, R(rotation));

        /// <summary>Konvexe Hülle beliebiger Punkte.</summary>
        public static ConvexCutter FromPoints(IEnumerable<Vector3> points)
        {
            var pts = new List<double[]>();
            foreach (var p in points) pts.Add(D(p));
            return ConvexCutter.FromPoints(pts);
        }

        /// <summary>Konvexe Hülle der Ecken eines (lesbaren) Meshes, mit localToWorld in Weltkoordinaten.</summary>
        public static ConvexCutter FromMesh(Mesh mesh, Matrix4x4 localToWorld)
        {
            var pts = new List<double[]>();
            foreach (var p in mesh.vertices) pts.Add(D(localToWorld.MultiplyPoint3x4(p)));
            return ConvexCutter.FromPoints(pts);
        }

        /// <summary>Mesh des Körpers mit flachen Flächen (Vorschau, Gizmos). Koordinaten wie der Körper.</summary>
        public static Mesh ToMesh(ConvexCutter c, Mesh reuse = null)
        {
            var mesh = reuse != null ? reuse : new Mesh { name = "Schneidkörper" };
            var v = new List<Vector3>(); var n = new List<Vector3>(); var idx = new List<int>();
            foreach (var f in c.Faces)
            {
                int b = v.Count;
                var nn = new Vector3((float)f.N[0], (float)f.N[1], (float)f.N[2]);
                foreach (int k in f.Loop) { var p = c.Verts[k]; v.Add(new Vector3((float)p[0], (float)p[1], (float)p[2])); n.Add(nn); }
                for (int i = 1; i + 1 < f.Loop.Length; i++) { idx.Add(b); idx.Add(b + i); idx.Add(b + i + 1); }
            }
            mesh.Clear();
            mesh.SetVertices(v); mesh.SetNormals(n); mesh.SetTriangles(idx, 0);
            mesh.RecalculateBounds();
            return mesh;
        }
    }
}
