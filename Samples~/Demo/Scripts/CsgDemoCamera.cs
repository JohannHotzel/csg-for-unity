using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>Kamera um einen Punkt: rechte Maustaste drehen, mittlere verschieben, Mausrad zoomen.</summary>
public class CsgDemoCamera : MonoBehaviour
{
    public Vector3 drehpunkt = new Vector3(0, 1, 0);
    public float abstand = 10;
    [Tooltip("Drehung um die Hochachse (Grad)")] public float azimut = 30;
    [Tooltip("Blick von oben (Grad)")] public float neigung = 22;

    void LateUpdate()
    {
        var maus = Mouse.current;
        if (maus != null)
        {
            var d = maus.delta.ReadValue();
            if (maus.rightButton.isPressed) { azimut += d.x * 0.25f; neigung = Mathf.Clamp(neigung - d.y * 0.25f, -10, 85); }
            if (maus.middleButton.isPressed) drehpunkt -= (transform.right * d.x + transform.up * d.y) * abstand * 0.0015f;
            float rad = maus.scroll.ReadValue().y;
            if (rad != 0) abstand = Mathf.Clamp(abstand * (1 - 0.08f * Mathf.Sign(rad)), 2, 40);
        }
        var q = Quaternion.Euler(neigung, azimut, 0);
        transform.SetPositionAndRotation(drehpunkt - q * Vector3.forward * abstand, q);
    }
}
