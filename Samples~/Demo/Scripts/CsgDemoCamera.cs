using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>Camera around a point: right mouse button orbits, middle pans, wheel zooms.</summary>
public class CsgDemoCamera : MonoBehaviour
{
    public Vector3 pivot = new Vector3(0, 1, 0);
    public float distance = 10;
    [Tooltip("Rotation around the vertical axis (degrees)")] public float yaw = 30;
    [Tooltip("Looking down from above (degrees)")] public float pitch = 22;

    void LateUpdate()
    {
        var mouse = Mouse.current;
        if (mouse != null)
        {
            var d = mouse.delta.ReadValue();
            if (mouse.rightButton.isPressed) { yaw += d.x * 0.25f; pitch = Mathf.Clamp(pitch - d.y * 0.25f, -10, 85); }
            if (mouse.middleButton.isPressed) pivot -= (transform.right * d.x + transform.up * d.y) * distance * 0.0015f;
            float wheel = mouse.scroll.ReadValue().y;
            if (wheel != 0) distance = Mathf.Clamp(distance * (1 - 0.08f * Mathf.Sign(wheel)), 2, 40);
        }
        var q = Quaternion.Euler(pitch, yaw, 0);
        transform.SetPositionAndRotation(pivot - q * Vector3.forward * distance, q);
    }
}
