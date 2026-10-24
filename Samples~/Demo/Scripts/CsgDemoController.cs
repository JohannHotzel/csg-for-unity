using CsgForUnity;
using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>Klick schneidet die gewählte Form aus dem getroffenen Körper. Tasten 1–4 wechseln die Form.</summary>
public class CsgDemoController : MonoBehaviour
{
    public enum Shape { Sphere, Box, Cylinder, Rock }
    public enum Orientation { World, Surface, Random }

    [System.Serializable] public class SphereSettings
    {
        public float radius = 0.35f;
        [Range(0, 2), Tooltip("Feinheit: 0 = 20 Flächen, 1 = 80, 2 = 320")] public int detail = 1;
    }
    [System.Serializable] public class BoxSettings
    {
        [Tooltip("Kantenlängen in Metern")] public Vector3 size = new Vector3(0.6f, 0.4f, 0.3f);
    }
    [System.Serializable] public class CylinderSettings
    {
        public float radius = 0.15f;
        [Tooltip("Länge entlang der eigenen y-Achse")] public float height = 1.5f;
        [Range(3, 48)] public int segments = 16;
    }
    [System.Serializable] public class RockSettings
    {
        public float radius = 0.4f;
        [Range(4, 40), Tooltip("Anzahl zufälliger Hüllpunkte; jeder Schnitt bekommt eine neue Form")] public int points = 16;
    }

    public Shape shape = Shape.Sphere;
    [Tooltip("World: achsparallel · Surface: y-Achse entlang der Oberflächennormale (Zylinder bohrt) · Random: zufällig je Schnitt")]
    public Orientation orientation = Orientation.Surface;
    [Tooltip("Wie tief der Mittelpunkt unter der getroffenen Oberfläche liegt (m)")]
    public float depth = 0;
    public SphereSettings sphere = new SphereSettings();
    public BoxSettings box = new BoxSettings();
    public CylinderSettings cylinder = new CylinderSettings();
    public RockSettings rock = new RockSettings();

    static readonly string[] names = { "Kugel", "Quader", "Zylinder", "Brocken" };
    InputAction click, shapeKeys;
    int seed;
    string last = "–";

    void OnEnable()
    {
        click = new InputAction(binding: "<Mouse>/leftButton");
        click.performed += _ => Cut();
        click.Enable();
        // Tasten 1–4: der Name der Taste ist die Ziffer
        shapeKeys = new InputAction(type: InputActionType.Button);
        for (int i = 1; i <= 4; i++) shapeKeys.AddBinding("<Keyboard>/" + i);
        shapeKeys.performed += c => shape = (Shape)(c.control.name[0] - '1');
        shapeKeys.Enable();
    }

    void OnDisable()
    {
        click.Dispose();
        shapeKeys.Dispose();
    }

    void Cut()
    {
        var ray = Camera.main.ScreenPointToRay(Mouse.current.position.ReadValue());
        if (!Physics.Raycast(ray, out var hit) || !hit.collider.TryGetComponent(out CsgBody body)) return;
        var rotation = orientation == Orientation.Surface ? Quaternion.FromToRotation(Vector3.up, hit.normal)
                     : orientation == Orientation.Random ? Random.rotation
                     : Quaternion.identity;
        var r = body.Subtract(MakeCutter(hit.point - hit.normal * depth, rotation));
        last = !r.Ok ? "abgelehnt: " + r.Reason
             : !r.Changed ? "nichts geschnitten"
             : $"{r.Ms:0.00} ms Schnitt + {body.LastRebuildMs:0.00} ms Mesh und Collider, {body.Triangles:N0} Dreiecke";
    }

    ConvexCutter MakeCutter(Vector3 center, Quaternion rotation)
    {
        switch (shape)
        {
            case Shape.Box: return CsgCutters.Box(center, box.size / 2, rotation);
            case Shape.Cylinder: return CsgCutters.Cylinder(center, cylinder.radius, cylinder.height, rotation, cylinder.segments);
            case Shape.Rock: return CsgCutters.Rock(center, rock.radius, ++seed, rotation, rock.points);
            default: return CsgCutters.Sphere(center, sphere.radius, rotation, sphere.detail);
        }
    }

    void OnGUI() => GUI.Box(new Rect(10, 10, 470, 62),
        $"Form: {names[(int)shape]}  (1 Kugel · 2 Quader · 3 Zylinder · 4 Brocken)\nLinks: schneiden · Rechts: drehen · Mitte: verschieben · Rad: zoomen\nLetzter Schnitt: {last}");
}
