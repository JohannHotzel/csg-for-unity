using System;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;
using UnityEngine.Rendering;

namespace CsgForUnity
{
    /// <summary>
    /// Macht das Netz am MeshFilter zerschneidbar: konvexe Körper (in Weltkoordinaten) werden abgezogen, das Ergebnis
    /// ist immer geschlossen. Submesh 0 = Außenfläche (bisheriges Material), Submesh 1 = Schnittflächen (innerMaterial).
    /// Das Quellnetz bleibt unverändert, der MeshFilter bekommt ein eigenes Netz.
    /// </summary>
    [RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
    [DisallowMultipleComponent]
    [AddComponentMenu("CSG for Unity/CSG Body")]
    public sealed class CsgBody : MonoBehaviour
    {
        [Tooltip("Material der Schnittflächen (Innenseite). Leer: Außenmaterial.")]
        public Material innerMaterial;
        [Tooltip("UVs der Schnittflächen: planar je Schnittebene, Position in Welt-Einheiten × Maßstab.")]
        public float innerUvScale = 1f;
        [Tooltip("Vorhandenen MeshCollider nach jedem Schnitt aktualisieren.")]
        public bool updateCollider = true;
        [Tooltip("Tangenten berechnen (nur für Normal Maps nötig).")]
        public bool computeTangents;
        [Tooltip("Toleranzen und Aufräumen; die Standardwerte passen für fast alle Fälle.")]
        public CutOptions options = new CutOptions();

        [SerializeField, HideInInspector] Mesh sourceMesh;
        CsgMesh model;
        Mesh output;
        Material outerMaterial;
        string error;

        static readonly List<CsgBody> all = new List<CsgBody>();
        /// <summary>Alle aktiven Körper (im Play Mode).</summary>
        public static IReadOnlyList<CsgBody> All => all;

        /// <summary>Nach jedem Schnittversuch an diesem Körper.</summary>
        public event Action<CsgBody, CutResult> Cut;

        /// <summary>Das Arbeitsnetz (double-Genauigkeit, lokale Koordinaten); null, wenn das Mesh unbrauchbar ist.</summary>
        public CsgMesh Model => Ensure() ? model : null;
        /// <summary>Warum der Körper nicht geschnitten werden kann (sonst null).</summary>
        public string Error => error;
        public Mesh Output => output;
        public int Triangles => model != null ? model.NTri : 0;
        /// <summary>Dauer des letzten Aufbaus des Unity-Netzes in ms (zusätzlich zur Schnittzeit).</summary>
        public double LastRebuildMs { get; private set; }

        void OnEnable() => all.Add(this);
        void OnDisable() => all.Remove(this);

        void OnDestroy()
        {
            if (output == null) return;
            if (Application.isPlaying) Destroy(output); else DestroyImmediate(output);
        }

        /// <summary>Netz übernehmen (geschieht sonst beim ersten Schnitt). false, wenn es nicht geschlossen oder nicht lesbar ist.</summary>
        public bool Ensure()
        {
            if (model != null) return true;
            if (error != null) return false;
            var mf = GetComponent<MeshFilter>();
            var mesh = mf.sharedMesh;
            if (mesh == null) return Fail("kein Mesh am MeshFilter");
            if (!mesh.isReadable) return Fail($"Mesh „{mesh.name}“ ist nicht lesbar (Read/Write in den Import-Einstellungen aktivieren)");
            if (sourceMesh == null) sourceMesh = mesh;
            // ein früher gespeichertes eigenes Ergebnis: Submesh 1 sind Schnittflächen
            model = Import(mesh, mesh != sourceMesh);
            var v = CsgAnalyze.Validate(model);
            if (!v.Ok) { model = null; return Fail($"Mesh „{mesh.name}“ ist nicht geschlossen ({v})"); }
            outerMaterial = GetComponent<MeshRenderer>().sharedMaterial;
            output = new Mesh { name = sourceMesh.name + " (CSG)" };
            output.MarkDynamic();
            mf.sharedMesh = output;
            Rebuild();
            return true;
        }

        bool Fail(string why)
        {
            error = why;
            UnityEngine.Debug.LogWarning($"CsgBody „{name}“: {why}", this);
            return false;
        }

        /// <summary>Zurück zum ursprünglichen Mesh (alle Schnitte verwerfen).</summary>
        public void ResetToSource()
        {
            if (sourceMesh == null) return;
            GetComponent<MeshFilter>().sharedMesh = sourceMesh;
            if (output != null) { if (Application.isPlaying) Destroy(output); else DestroyImmediate(output); }
            output = null; model = null; error = null;
            var mc = GetComponent<MeshCollider>();
            if (mc != null && updateCollider) mc.sharedMesh = sourceMesh;
            Ensure();
        }

        /// <summary>Konvexen Körper in Weltkoordinaten abziehen.</summary>
        public CutResult Subtract(ConvexCutter worldCutter)
        {
            if (!Ensure()) return new CutResult { Reason = error };
            if (!Overlaps(GetComponent<MeshRenderer>().bounds, worldCutter)) return new CutResult { Ok = true };
            ConvexCutter local;
            try { local = ToLocal(worldCutter); }
            catch (Exception e) { return new CutResult { Reason = "Körper entartet: " + e.Message }; }
            return SubtractLocal(local);
        }

        /// <summary>Konvexen Körper in lokalen Koordinaten dieses Objekts abziehen.</summary>
        public CutResult SubtractLocal(ConvexCutter localCutter)
        {
            if (!Ensure()) return new CutResult { Reason = error };
            var r = Csg.Subtract(model, localCutter, options);
            if (r.Ok && r.Changed)
            {
                // unbenutzte Ecken sammeln sich an (Schnittpunkte entfernter Dreiecke): gelegentlich verdichten
                if (model.Nv > 4096 && model.Nv > 3 * model.NTri) model.Compact();
                Rebuild();
            }
            Cut?.Invoke(this, r);
            return r;
        }

        /// <summary>Körper von allen aktiven CsgBodies abziehen, die er berührt. Liefert die Anzahl geänderter Körper.</summary>
        public static int SubtractAll(ConvexCutter worldCutter)
        {
            int n = 0;
            foreach (var b in all.ToArray())
                if (b != null && b.isActiveAndEnabled && b.Subtract(worldCutter).Changed) n++;
            return n;
        }

        static bool Overlaps(Bounds b, ConvexCutter c)
        {
            Vector3 mn = b.min, mx = b.max;
            double e = 1e-4 * (b.size.magnitude + 1e-6);
            return !(c.Max[0] < mn.x - e || c.Min[0] > mx.x + e || c.Max[1] < mn.y - e || c.Min[1] > mx.y + e || c.Max[2] < mn.z - e || c.Min[2] > mx.z + e);
        }

        ConvexCutter ToLocal(ConvexCutter c)
        {
            var M = transform.worldToLocalMatrix;
            var src = c.Points ?? c.Verts;
            var pts = new double[src.Length][];
            for (int i = 0; i < src.Length; i++)
            {
                double x = src[i][0], y = src[i][1], z = src[i][2];
                pts[i] = new[]
                {
                    M.m00 * x + M.m01 * y + M.m02 * z + M.m03,
                    M.m10 * x + M.m11 * y + M.m12 * z + M.m13,
                    M.m20 * x + M.m21 * y + M.m22 * z + M.m23,
                };
            }
            return ConvexCutter.FromPoints(pts);
        }

        static CsgMesh Import(Mesh mesh, bool secondIsInner)
        {
            var V = mesh.vertices; var N = mesh.normals; var UV = mesh.uv;
            var pos = new double[3 * V.Length];
            for (int i = 0; i < V.Length; i++) { pos[3 * i] = V[i].x; pos[3 * i + 1] = V[i].y; pos[3 * i + 2] = V[i].z; }
            double[] nrm = null, uvs = null;
            if (N.Length == V.Length) { nrm = new double[3 * V.Length]; for (int i = 0; i < V.Length; i++) { nrm[3 * i] = N[i].x; nrm[3 * i + 1] = N[i].y; nrm[3 * i + 2] = N[i].z; } }
            if (UV.Length == V.Length) { uvs = new double[2 * V.Length]; for (int i = 0; i < V.Length; i++) { uvs[2 * i] = UV[i].x; uvs[2 * i + 1] = UV[i].y; } }
            var idx = new List<int>(); var mat = new List<int>();
            for (int s = 0; s < mesh.subMeshCount; s++)
            {
                if (mesh.GetTopology(s) != MeshTopology.Triangles) continue;
                var t = mesh.GetTriangles(s);
                idx.AddRange(t);
                int m = secondIsInner && s == 1 ? 1 : 0;
                for (int i = 0; i < t.Length / 3; i++) mat.Add(m);
            }
            return CsgMesh.FromIndexed(pos, idx.ToArray(), nrm, uvs, triMat: mat.ToArray());
        }

        // Ecken mit gleicher Position und (fast) gleichen Attributen teilen sich einen Unity-Vertex. Je Arbeitsnetz-Ecke
        // eine Kette ihrer Unity-Vertices (meist 1–3): erst über den Patch suchen (gleiche Attribute ohne Rechnen),
        // dann über die gerundeten Attribute (glatte Flächen aus vielen Patches).
        readonly List<Vector3> vbuf = new List<Vector3>(), nbuf = new List<Vector3>();
        readonly List<Vector2> uvbuf = new List<Vector2>();
        readonly List<int> ibuf0 = new List<int>(), ibuf1 = new List<int>();
        int[] head = new int[0], next = new int[0], opatch = new int[0], qn = new int[0];
        long[] quv = new long[0];
        readonly Dictionary<int, double[]> capFrames = new Dictionary<int, double[]>();
        readonly double[] attr = new double[5];

        int VertexFor(int v, int pid, Patch pa, double[] fr, double sx, double sy, double sz)
        {
            for (int o = head[v]; o >= 0; o = next[o]) if (opatch[o] == pid) return o;
            var P = model.Pos;
            double x = P[3 * v], y = P[3 * v + 1], z = P[3 * v + 2];
            if (fr != null)
            {
                // planare UVs im Weltmaßstab: nicht verzerrt bei ungleichmäßig skalierten Objekten
                double qx = x * sx, qy = y * sy, qz = z * sz, k = innerUvScale;
                attr[0] = pa.N[0]; attr[1] = pa.N[1]; attr[2] = pa.N[2];
                attr[3] = (qx * fr[0] + qy * fr[1] + qz * fr[2]) * k;
                attr[4] = (qx * fr[3] + qy * fr[4] + qz * fr[5]) * k;
            }
            else pa.Attr(x, y, z, attr);
            int n0 = (int)Rnd(attr[0] * 2048), n1 = (int)Rnd(attr[1] * 2048), n2 = (int)Rnd(attr[2] * 2048);
            long u0 = Rnd(attr[3] * 1e4), u1 = Rnd(attr[4] * 1e4);
            for (int o = head[v]; o >= 0; o = next[o])
                if (qn[3 * o] == n0 && qn[3 * o + 1] == n1 && qn[3 * o + 2] == n2 && quv[2 * o] == u0 && quv[2 * o + 1] == u1) return o;
            int id = vbuf.Count;
            if (id >= next.Length)
            {
                int c = Math.Max(1024, next.Length * 2);
                Array.Resize(ref next, c); Array.Resize(ref opatch, c); Array.Resize(ref qn, 3 * c); Array.Resize(ref quv, 2 * c);
            }
            next[id] = head[v]; head[v] = id; opatch[id] = pid;
            qn[3 * id] = n0; qn[3 * id + 1] = n1; qn[3 * id + 2] = n2; quv[2 * id] = u0; quv[2 * id + 1] = u1;
            vbuf.Add(new Vector3((float)x, (float)y, (float)z));
            nbuf.Add(new Vector3((float)attr[0], (float)attr[1], (float)attr[2]));
            uvbuf.Add(new Vector2((float)attr[3], (float)attr[4]));
            return id;
        }

        // Runden ohne Math.Round (unter Mono ein Laufzeitaufruf, hier bis zu fünfmal je Dreiecksecke)
        static long Rnd(double x) => (long)(x >= 0 ? x + 0.5 : x - 0.5);

        /// <summary>Unity-Netz aus dem Arbeitsnetz neu aufbauen (nach jedem Schnitt automatisch).</summary>
        public void Rebuild()
        {
            if (model == null) return;
            var sw = Stopwatch.StartNew();
            var m = model; var T = m.Tri;
            vbuf.Clear(); nbuf.Clear(); uvbuf.Clear(); ibuf0.Clear(); ibuf1.Clear(); capFrames.Clear();
            if (head.Length < m.Nv) head = new int[Math.Max(m.Nv, 2 * head.Length)];
            for (int i = 0; i < m.Nv; i++) head[i] = -1;
            var S = transform.lossyScale;
            double sx = Math.Abs(S.x) > 1e-12 ? Math.Abs(S.x) : 1, sy = Math.Abs(S.y) > 1e-12 ? Math.Abs(S.y) : 1, sz = Math.Abs(S.z) > 1e-12 ? Math.Abs(S.z) : 1;
            for (int t = 0; t < m.NSlots; t++)
            {
                if (T[3 * t] < 0) continue;
                int pid = m.TPatch[t];
                var pa = m.Patches[pid];
                double[] fr = null;
                if (pa.Mat == 1 && !capFrames.TryGetValue(pid, out fr)) capFrames[pid] = fr = CapFrame(pa.N, sx, sy, sz);
                var ib = pa.Mat == 1 ? ibuf1 : ibuf0;
                ib.Add(VertexFor(T[3 * t], pid, pa, fr, sx, sy, sz));
                ib.Add(VertexFor(T[3 * t + 1], pid, pa, fr, sx, sy, sz));
                ib.Add(VertexFor(T[3 * t + 2], pid, pa, fr, sx, sy, sz));
            }
            output.Clear();
            output.indexFormat = vbuf.Count > 65535 ? IndexFormat.UInt32 : IndexFormat.UInt16;
            output.SetVertices(vbuf);
            output.SetNormals(nbuf);
            output.SetUVs(0, uvbuf);
            output.subMeshCount = 2;
            output.SetTriangles(ibuf0, 0, false);
            output.SetTriangles(ibuf1, 1, false);
            output.RecalculateBounds();
            if (computeTangents) output.RecalculateTangents();
            GetComponent<MeshRenderer>().sharedMaterials = new[] { outerMaterial, innerMaterial != null ? innerMaterial : outerMaterial };
            var mc = GetComponent<MeshCollider>();
            if (mc != null && updateCollider)
            {
                mc.sharedMesh = null;
                if (m.NTri > 0) mc.sharedMesh = output;
            }
            LastRebuildMs = sw.Elapsed.TotalMilliseconds;
        }

        // Rahmen (U, V) der Schnittebene in skalierten Koordinaten q = S·p: dort hat die Ebene die Normale n / S
        static double[] CapFrame(double[] n, double sx, double sy, double sz)
        {
            double ax = n[0] / sx, ay = n[1] / sy, az = n[2] / sz, l = Math.Sqrt(ax * ax + ay * ay + az * az);
            ax /= l; ay /= l; az /= l;
            double hx = 0, hy = 0, hz = 0;
            if (Math.Abs(ax) < 0.6) hx = 1; else if (Math.Abs(ay) < 0.6) hy = 1; else hz = 1;
            double ux = ay * hz - az * hy, uy = az * hx - ax * hz, uz = ax * hy - ay * hx, lu = Math.Sqrt(ux * ux + uy * uy + uz * uz);
            ux /= lu; uy /= lu; uz /= lu;
            return new[] { ux, uy, uz, ay * uz - az * uy, az * ux - ax * uz, ax * uy - ay * ux };
        }
    }
}
