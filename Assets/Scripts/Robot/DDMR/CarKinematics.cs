using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class CarKinematics : MonoBehaviour
{
    [Header("Drive Parameters")]
    public float maxTorque = 50f;          // max wheel torque
    public float maxSpeed = 5f;             // m/s
    public float turnTorque = 150f;         // torque for in-place turn
    public float slowDownRadius = 1f;       // start slowing down
    public float stopThreshold = 0.2f;      // distance to consider reached
    public float turnThreshold = 0.5f;      // distance to switch to in-place turn

    private Rigidbody rb;
    private List<WheelControl> wheels; 

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        wheels = new List<WheelControl>(GetComponentsInChildren<WheelControl>());
    }

    private void Start()
    {
        // Debug.Log($"Initial rotation: {transform.rotation.eulerAngles}");
        // Debug.Log($"Initial forward: {transform.forward}");
    }

    public IEnumerator NavigateToWaypoints(List<Vector3> worldWaypoints)
    {
        // for each waypoint, rotate → drive → brake
        foreach (var wp in worldWaypoints)
        {
            // Debug.Log($"Navigating to target: {wp}");
            Vector2 wp2D = new Vector2(wp.x, wp.z);
            yield return StartCoroutine(RotateToTarget(wp2D));
            yield return StartCoroutine(DriveToTarget(wp2D));
        }
        // Debug.Log("All waypoints reached. Applying brakes.");
        ApplyBrake();
    }
    private Vector2 getPosition()
    {
        // return position in XZ plane
        return new Vector2(transform.position.x, transform.position.z);
    }

    private Vector2 getForward()
    {
        // return forward direction in XZ plane
        return new Vector2(transform.forward.x, transform.forward.z);
    }

    private IEnumerator RotateToTarget(Vector2 target)
    {
        // Debug.Log($"Rotating to target: {target}");
        while (true)
        {
            // Calculate direction to target in XZ plane
            Vector2 dirToTarget = target - getPosition();
            float distance = dirToTarget.magnitude;
            
            // Get current forward direction in XZ plane
            Vector2 currentForward = getForward();
            
            // Calculate angle between current forward and target direction
            float desiredAngle = Vector2.SignedAngle(currentForward, dirToTarget);
            
            // Debug.Log($"Current forward: {currentForward}, Dir to target: {dirToTarget}, Angle: {desiredAngle}");
            
            // Stop rotating if angle is small enough or we're very close to target
            if (Mathf.Abs(desiredAngle) < 1f || distance < turnThreshold) 
                break;

            // Determine rotation direction
            float sign = Mathf.Sign(desiredAngle);
            // Debug.Log($"Rotating: Current rotation: {transform.rotation.eulerAngles}, Desired angle: {desiredAngle}");
            // transform.rotation = Quaternion.RotateTowards(transform.rotation, Quaternion.Euler(0, transform.rotation.eulerAngles.y + desiredAngle, 0), turnTorque * Time.deltaTime);
            SetWheelTorque(-sign * turnTorque, sign * turnTorque); 
            yield return new WaitForFixedUpdate();
        }
        
        ApplyBrake();
        yield return new WaitForSeconds(0.1f);
    }

    private IEnumerator DriveToTarget(Vector2 target)
    {
        // Debug.Log($"Driving to target: {target}");
        while (true)
        {
            Vector2 dir = (target - getPosition());
            float distance = dir.magnitude;
            // Debug.Log($"Distance to target: {distance}, dir: {dir}, Transform.position: {transform.position}");
            if (distance < stopThreshold) break;

            // Speed factor: 1 at far, <1 when close
            float speedFactor = Mathf.Clamp01(distance / slowDownRadius);
            float torque = maxTorque * speedFactor;

            // Debug.Log($"Driving: Distance = {distance}, Speed Factor = {speedFactor}, Torque = {torque}");
            // transform.position = Vector3.MoveTowards(transform.position, target, maxSpeed * Time.deltaTime);
            SetWheelTorque(torque, torque, true);

            yield return new WaitForFixedUpdate();
        }
        // Debug.Log("Target reached. Applying brake.");
        ApplyBrake();
        yield return new WaitForSeconds(0.1f);
    }

    private void SetWheelTorque(float leftTorque, float rightTorque, bool movingForward = false)
    {
        // Debug.Log($"Setting wheel torque: Left = {leftTorque}, Right = {rightTorque}");
        foreach (var w in wheels)
        {
            if (w.leftWheel) w.WheelCollider.motorTorque = leftTorque;
            else if (w.rightWheel) w.WheelCollider.motorTorque = rightTorque;

            // reduce brakes when accelerating
            if (movingForward) w.WheelCollider.brakeTorque = 0f; 
            // Debug.Log($"Wheel {w.name}: Left = {w.leftWheel}, Right = {w.rightWheel}, Torque = {w.WheelCollider.motorTorque}");
        }
    }

    private void ApplyBrake()
    {
        // Debug.Log("Applying brakes.");
        foreach (var w in wheels)
        {
            w.WheelCollider.motorTorque = 0f;
            w.WheelCollider.brakeTorque = maxTorque * 10f; // Increase brake torque for faster stopping
        }
    }

    // PUBLIC UTILITIES

    public IEnumerator rotate(float angle)
    {
        float sign = Mathf.Sign(angle);
        float targetAngle = transform.eulerAngles.y + angle; // Calculate the target angle relative to the current rotation

        while (true)
        {
            float currentAngle = transform.eulerAngles.y;
            float angleDifference = Mathf.DeltaAngle(currentAngle, targetAngle); // Calculate the shortest angle difference

            if (Mathf.Abs(angleDifference) < 1f) // Stop rotating if the angle difference is small enough
                break;

            SetWheelTorque(sign * turnTorque, -sign * turnTorque);

            yield return new WaitForFixedUpdate();
        }

        ApplyBrake();
        yield return new WaitForSeconds(0.1f); // Small delay to ensure the car stabilizes
    }
    public void steer(float angle)
    {
        float sign = Mathf.Sign(angle);
        SetWheelTorque(sign * turnTorque, -sign * turnTorque);
    }

    public void brake(float brakeForce)
    {
        rb.velocity = Vector3.Lerp(rb.velocity, Vector3.zero, brakeForce * Time.deltaTime);
    }

    public void drive()
    {
        SetWheelTorque(maxTorque, maxTorque, true);
    }

    public void driveBackward()
    {
        SetWheelTorque(-maxTorque, -maxTorque, true);
    }
}   