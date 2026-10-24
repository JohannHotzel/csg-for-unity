using CsgForUnity;
using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>Clicking cuts the selected shape out of the body under the cursor. Keys 1–4 switch the shape.</summary>
public class CsgDemoController : MonoBehaviour
{
    public enum Shape { Sphere, Box, Cylinder, Rock }
    public enum Orientation { World, Surface, Random }

    [System.Serializable] public class SphereSettings
    {
        public float radius = 0.35f;
        [Range(0, 2), Tooltip("0 = 20 faces, 1 = 80, 2 = 320")] public int detail = 1;
    }
    [System.Serializable] public class BoxSettings
    {
        [Tooltip("Edge lengths in metres")] public Vector3 size = new Vector3(0.6f, 0.4f, 0.3f);
    }
    [System.Serializable] public class CylinderSettings
    {
        public float radius = 0.15f;
        [Tooltip("Length along its own y axis")] public float height = 1.5f;
        [Range(3, 48)] public int segments = 16;
    }
    [System.Serializable] public class RockSettings
    {
        public float radius = 0.4f;
        [Range(4, 40), Tooltip("Number of random hull points; every cut gets a new shape")] public int points = 16;
    }

    public Shape shape = Shape.Sphere;
    [Tooltip("World: axis-aligned · Surface: y axis along the surface normal (a cylinder drills) · Random: random per cut")]
    public Orientation orientation = Orientation.Surface;
    [Tooltip("How far below the hit surface the centre lies (m)")]
    public float depth = 0;
    public SphereSettings sphere = new SphereSettings();
    public BoxSettings box = new BoxSettings();
    public CylinderSettings cylinder = new CylinderSettings();
    public RockSettings rock = new RockSettings();

    InputAction click, shapeKeys;
    int seed;
    string last = "–";

    void OnEnable()
    {
        click = new InputAction(binding: "<Mouse>/leftButton");
        click.performed += _ => Cut();
        click.Enable();
        // keys 1–4: the key's name is the digit
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
        last = !r.Ok ? "rejected: " + r.Reason
             : !r.Changed ? "nothing cut"
             : $"{r.Ms:0.00} ms cut + {body.LastRebuildMs:0.00} ms mesh and collider, {body.Triangles:N0} triangles";
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
        $"Shape: {shape}  (1 sphere · 2 box · 3 cylinder · 4 rock)\nLeft: cut · Right: orbit · Middle: pan · Wheel: zoom\nLast cut: {last}");
}
