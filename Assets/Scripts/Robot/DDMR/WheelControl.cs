using UnityEngine;

[RequireComponent(typeof(WheelCollider))]
public class WheelControl : MonoBehaviour
{
    public Transform wheelModel;

    [HideInInspector] public WheelCollider WheelCollider;
    // Create properties for the RobotControl script
    // (You should enable/disable these via the 
    // Editor Inspector window)
    public bool leftWheel;
    public bool rightWheel;

    // Start is called before the first frame update
    private void Awake()
    {
        WheelCollider = GetComponent<WheelCollider>();
    }

    // Update is called once per frame
    void Update()
    {
        // Get the Wheel collider's world pose values and
        // use them to set the wheel model's position and rotation
        WheelCollider.GetWorldPose(out Vector3 pos, out Quaternion rot);
        wheelModel.position = pos;
        wheelModel.rotation = rot;
    }
}
