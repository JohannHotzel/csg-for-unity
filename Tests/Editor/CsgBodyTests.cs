using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;

namespace CsgForUnity.Tests
{
    public class CsgBodyTests
    {
        readonly List<Object> trash = new List<Object>();

        [TearDown]
        public void TearDown()
        {
            foreach (var o in trash) if (o != null) Object.DestroyImmediate(o);
            trash.Clear();
        }

        CsgBody Make(PrimitiveType type, Vector3 pos, Vector3 scale, Quaternion rot)
        {
            var go = GameObject.CreatePrimitive(type);
            trash.Add(go);
            go.transform.SetPositionAndRotation(pos, rot);
            go.transform.localScale = scale;
            var b = go.AddComponent<CsgBody>();
            var inner = new Material(Shader.Find("Hidden/InternalErrorShader")) { name = "Innen" };
            trash.Add(inner);
            b.innerMaterial = inner;
            return b;
        }

        // Volumen des Unity-Netzes in Weltkoordinaten (beide Submeshes)
        static double WorldVolume(CsgBody b)
        {
            var m = b.Output; var V = m.vertices; var M = b.transform.localToWorldMatrix;
            double vol = 0;
            for (int s = 0; s < m.subMeshCount; s++)
            {
                var T = m.GetTriangles(s);
                for (int i = 0; i < T.Length; i += 3)
                {
                    Vector3 a = M.MultiplyPoint3x4(V[T[i]]), c1 = M.MultiplyPoint3x4(V[T[i + 1]]), c2 = M.MultiplyPoint3x4(V[T[i + 2]]);
                    vol += Vector3.Dot(a, Vector3.Cross(c1, c2));
                }
            }
            return vol / 6;
        }

        [Test]
        public void PrimitivesAreClosed([Values(PrimitiveType.Cube, PrimitiveType.Sphere, PrimitiveType.Cylinder, PrimitiveType.Capsule)] PrimitiveType type)
        {
            var b = Make(type, Vector3.zero, Vector3.one, Quaternion.identity);
            Assert.IsTrue(b.Ensure(), b.Error);
            Assert.IsTrue(CsgAnalyze.Validate(b.Model).Ok);
        }

        [Test]
        public void OpenMeshIsRejected()
        {
            var b = Make(PrimitiveType.Quad, Vector3.zero, Vector3.one, Quaternion.identity);
            Assert.IsFalse(b.Ensure());
            StringAssert.Contains("nicht geschlossen", b.Error);
            Assert.IsFalse(b.Subtract(CsgCutters.Sphere(Vector3.zero, 0.3f, Quaternion.identity)).Ok);
        }

        [Test]
        public void CornerOfScaledCube()
        {
            // Würfel 4 × 2 × 1 (Welt), Ecke 0.5³ abziehen
            var b = Make(PrimitiveType.Cube, new Vector3(10, 0, 0), new Vector3(4, 2, 1), Quaternion.identity);
            var r = b.Subtract(CsgCutters.Box(new Vector3(12, 1, 0.5f), Vector3.one * 0.5f, Quaternion.identity));
            Assert.IsTrue(r.Ok && r.Changed, r.ToString());
            Assert.IsTrue(CsgAnalyze.Validate(b.Model).Ok);
            Assert.AreEqual(8 - 0.125, WorldVolume(b), 1e-4);
            Assert.AreEqual(2, b.Output.subMeshCount);
            Assert.Greater(b.Output.GetTriangles(1).Length, 0, "Schnittflächen im Submesh 1");
            var mats = b.GetComponent<MeshRenderer>().sharedMaterials;
            Assert.AreEqual(2, mats.Length);
            Assert.AreSame(b.innerMaterial, mats[1]);
        }

        [Test]
        public void CapUvsAreWorldScaled()
        {
            // Schnittfläche quer durch einen stark gestreckten Würfel: UV-Ausdehnung = Weltausdehnung
            var b = Make(PrimitiveType.Cube, Vector3.zero, new Vector3(8, 1, 1), Quaternion.identity);
            b.Subtract(CsgCutters.Box(new Vector3(0, 0, 1), new Vector3(10, 10, 1), Quaternion.identity));
            var m = b.Output; var uv = m.uv; var T = m.GetTriangles(1);
            Assert.Greater(T.Length, 0);
            Vector2 mn = Vector2.positiveInfinity, mx = Vector2.negativeInfinity;
            foreach (int i in T) { mn = Vector2.Min(mn, uv[i]); mx = Vector2.Max(mx, uv[i]); }
            var ext = mx - mn;
            Assert.AreEqual(8, Mathf.Max(ext.x, ext.y), 1e-3);
            Assert.AreEqual(1, Mathf.Min(ext.x, ext.y), 1e-3);
        }

        [Test]
        public void RotatedCutsKeepMeshClosed()
        {
            var b = Make(PrimitiveType.Sphere, new Vector3(0, 1, 0), Vector3.one * 2, Quaternion.Euler(10, 20, 30));
            var rnd = new System.Random(5);
            Assert.IsTrue(b.Ensure(), b.Error);
            double v0 = WorldVolume(b);
            for (int i = 0; i < 40; i++)
            {
                var p = new Vector3(0, 1, 0) + Random(rnd) * 0.9f;
                var rot = Quaternion.Euler((float)rnd.NextDouble() * 360, (float)rnd.NextDouble() * 360, 0);
                var C = (i % 3) switch
                {
                    0 => CsgCutters.Sphere(p, 0.25f, rot),
                    1 => CsgCutters.Box(p, new Vector3(0.3f, 0.1f, 0.2f), rot),
                    _ => CsgCutters.Rock(p, 0.3f, i, rot),
                };
                var r = b.Subtract(C);
                Assert.IsTrue(r.Ok, r.ToString());
                Assert.IsTrue(CsgAnalyze.Validate(b.Model).Ok, $"Schnitt {i}");
            }
            double v1 = WorldVolume(b);
            Assert.Less(v1, v0);
            // Unity-Netz und Arbeitsnetz haben dasselbe Volumen (lokal × Skalierung³)
            Assert.AreEqual(CsgAnalyze.Volume(b.Model) * 8, v1, 1e-3 * v0);
        }

        static Vector3 Random(System.Random r) => new Vector3((float)r.NextDouble() - 0.5f, (float)r.NextDouble() - 0.5f, (float)r.NextDouble() - 0.5f).normalized;

        [Test]
        public void CutThroughSplitsAndResets()
        {
            var b = Make(PrimitiveType.Cylinder, Vector3.zero, new Vector3(0.5f, 2, 0.5f), Quaternion.identity);
            var r = b.Subtract(CsgCutters.Box(Vector3.zero, new Vector3(1, 0.1f, 1), Quaternion.identity));
            Assert.IsTrue(r.Ok && r.Changed, r.ToString());
            Assert.AreEqual(2, CsgAnalyze.Components(b.Model).Count, "zwei Teile");
            b.ResetToSource();
            Assert.AreEqual(1, CsgAnalyze.Components(b.Model).Count);
            Assert.AreEqual(0, b.Output.GetTriangles(1).Length);
        }

        [Test]
        public void ReimportKeepsInnerFaces()
        {
            // ein schon geschnittenes Netz erneut übernehmen (z. B. nach dem Laden der Szene): Schnittflächen bleiben innen
            var b = Make(PrimitiveType.Cube, Vector3.zero, Vector3.one, Quaternion.identity);
            b.Subtract(CsgCutters.Sphere(new Vector3(0.5f, 0.5f, 0.5f), 0.4f, Quaternion.identity));
            int inner = b.Output.GetTriangles(1).Length;
            var go = new GameObject("Kopie", typeof(MeshFilter), typeof(MeshRenderer));
            trash.Add(go);
            go.GetComponent<MeshFilter>().sharedMesh = Object.Instantiate(b.Output);
            trash.Add(go.GetComponent<MeshFilter>().sharedMesh);
            var c = go.AddComponent<CsgBody>();
            // wie nach dem Laden einer gespeicherten Szene: die gemerkte Quelle ist ein anderes Mesh als das am MeshFilter
            var original = new Mesh { name = "Original" };
            trash.Add(original);
            typeof(CsgBody).GetField("sourceMesh", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance).SetValue(c, original);
            Assert.IsTrue(c.Ensure(), c.Error);
            Assert.AreEqual(inner, c.Output.GetTriangles(1).Length);
        }
    }
}
