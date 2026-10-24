using CsgForUnity;
using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>Klick schneidet die gewählte Form aus dem getroffenen Körper. Tasten 1–4 wechseln die Form.</summary>
public class CsgDemoController : MonoBehaviour
{
    public enum Form { Kugel, Quader, Zylinder, Brocken }
    public enum Ausrichtung { Welt, [InspectorName("Oberfläche")] Oberflaeche, Zufall }

    [System.Serializable] public class KugelEinstellungen
    {
        public float radius = 0.35f;
        [Range(0, 2), Tooltip("0 = 20 Flächen, 1 = 80, 2 = 320")] public int feinheit = 1;
    }
    [System.Serializable] public class QuaderEinstellungen
    {
        [Tooltip("Kantenlängen in Metern")] public Vector3 abmessungen = new Vector3(0.6f, 0.4f, 0.3f);
    }
    [System.Serializable] public class ZylinderEinstellungen
    {
        public float radius = 0.15f;
        [Tooltip("Länge entlang der eigenen y-Achse")] public float höhe = 1.5f;
        [Range(3, 48)] public int segmente = 16;
    }
    [System.Serializable] public class BrockenEinstellungen
    {
        public float radius = 0.4f;
        [Range(4, 40), Tooltip("Anzahl zufälliger Hüllpunkte; jeder Schnitt bekommt eine neue Form")] public int punkte = 16;
    }

    public Form form = Form.Kugel;
    [Tooltip("Welt: achsparallel · Oberfläche: y-Achse entlang der Oberflächennormale (Zylinder bohrt) · Zufall: zufällig je Schnitt")]
    public Ausrichtung ausrichtung = Ausrichtung.Oberflaeche;
    [Tooltip("Wie tief der Mittelpunkt unter der getroffenen Oberfläche liegt (m)")]
    public float tiefe = 0;
    public KugelEinstellungen kugel = new KugelEinstellungen();
    public QuaderEinstellungen quader = new QuaderEinstellungen();
    public ZylinderEinstellungen zylinder = new ZylinderEinstellungen();
    public BrockenEinstellungen brocken = new BrockenEinstellungen();

    InputAction klick, formTasten;
    int saat;
    string letzter = "–";

    void OnEnable()
    {
        klick = new InputAction(binding: "<Mouse>/leftButton");
        klick.performed += _ => Schneiden();
        klick.Enable();
        // Tasten 1–4: der Name der Taste ist die Ziffer
        formTasten = new InputAction(type: InputActionType.Button);
        for (int i = 1; i <= 4; i++) formTasten.AddBinding("<Keyboard>/" + i);
        formTasten.performed += c => form = (Form)(c.control.name[0] - '1');
        formTasten.Enable();
    }

    void OnDisable()
    {
        klick.Dispose();
        formTasten.Dispose();
    }

    void Schneiden()
    {
        var strahl = Camera.main.ScreenPointToRay(Mouse.current.position.ReadValue());
        if (!Physics.Raycast(strahl, out var treffer) || !treffer.collider.TryGetComponent(out CsgBody körper)) return;
        var drehung = ausrichtung == Ausrichtung.Oberflaeche ? Quaternion.FromToRotation(Vector3.up, treffer.normal)
                    : ausrichtung == Ausrichtung.Zufall ? Random.rotation
                    : Quaternion.identity;
        var r = körper.Subtract(Schneidform(treffer.point - treffer.normal * tiefe, drehung));
        letzter = !r.Ok ? "abgelehnt: " + r.Reason
                : !r.Changed ? "nichts geschnitten"
                : $"{r.Ms:0.00} ms Schnitt + {körper.LastRebuildMs:0.00} ms Mesh und Collider, {körper.Triangles:N0} Dreiecke";
    }

    ConvexCutter Schneidform(Vector3 mitte, Quaternion drehung)
    {
        switch (form)
        {
            case Form.Quader: return CsgCutters.Box(mitte, quader.abmessungen / 2, drehung);
            case Form.Zylinder: return CsgCutters.Cylinder(mitte, zylinder.radius, zylinder.höhe, drehung, zylinder.segmente);
            case Form.Brocken: return CsgCutters.Rock(mitte, brocken.radius, ++saat, drehung, brocken.punkte);
            default: return CsgCutters.Sphere(mitte, kugel.radius, drehung, kugel.feinheit);
        }
    }

    void OnGUI() => GUI.Box(new Rect(10, 10, 470, 62),
        $"Form: {form}  (1 Kugel · 2 Quader · 3 Zylinder · 4 Brocken)\nLinks: schneiden · Rechts: drehen · Mitte: verschieben · Rad: zoomen\nLetzter Schnitt: {letzter}");
}
