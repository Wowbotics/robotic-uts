using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class RobotControlSimple : MonoBehaviour
{
    [Header("Drive Parameters")]
    public float maxTorque = 200f;          // max wheel torque
    public float maxSpeed = 5f;             // m/s
    public float turnTorque = 150f;         // torque for in-place turn
    public float slowDownRadius = 1f;       // start slowing down
    public float stopThreshold = 0.2f;      // distance to consider reached
    public float turnThreshold = 0.5f;      // distance to switch to in-place turn

    [Header("Waypoints")]
    public List<Vector3> waypoints = new List<Vector3>();

    private Rigidbody rb;
    private List<WheelControl> wheels;

    private void Start()
    {
        rb = GetComponent<Rigidbody>();
        wheels = new List<WheelControl>(GetComponentsInChildren<WheelControl>());

        //  Set initial forward direction to (0,0,1)
        // transform.forward = Vector3.forward;  // This is equivalent to (0,0,1)

    
        Debug.Log($"Initial rotation: {transform.rotation.eulerAngles}");
        Debug.Log($"Initial forward: {transform.forward}");
        StartCoroutine(NavigateRoutine());
    }

    private IEnumerator NavigateRoutine()
    {
        foreach (Vector3 _ in waypoints)
        {
            Vector3 target = _; 
            target.y = transform.position.y; // Keep the same height
            Debug.Log($"Navigating to target: {target}");
            // 1. Rotate to face target
            yield return StartCoroutine(RotateToTarget(target));
            // 2. Drive straight to target
            yield return StartCoroutine(DriveToTarget(target));
        }
        // All done: apply brakes
        Debug.Log("All waypoints reached. Applying brakes.");
        ApplyBrake();
    }

    private IEnumerator RotateToTarget(Vector3 target)
    {
        Debug.Log($"Rotating to target: {target}");
        while (true)
        {
            // Calculate direction to target in XZ plane only
            Vector3 dirToTarget = target - transform.position;
            dirToTarget.y = 0; // Ignore height difference
            float distance = dirToTarget.magnitude;
            
            // Get current forward direction in XZ plane
            Vector3 currentForward = transform.forward;
            currentForward.y = 0;
            
            // Calculate angle between current forward and target direction
            float desiredAngle = Vector3.SignedAngle(currentForward, dirToTarget, Vector3.up);
            
            Debug.Log($"Current forward: {currentForward}, Dir to target: {dirToTarget}, Angle: {desiredAngle}");
            
            // Stop rotating if angle is small enough or we're very close to target
            if (Mathf.Abs(desiredAngle) < 1f || distance < turnThreshold) 
                break;

            // Determine rotation direction
            float sign = Mathf.Sign(desiredAngle);
            transform.rotation = Quaternion.RotateTowards(transform.rotation, Quaternion.Euler(0, transform.rotation.eulerAngles.y + desiredAngle, 0), turnTorque * Time.deltaTime);
            // can use real torque but hard 
            // SetWheelTorque(-sign * turnTorque, sign * turnTorque); // TODO: uncomment 

            yield return new WaitForFixedUpdate();
        }
        
        ApplyBrake();
        yield return new WaitForSeconds(0.1f);
    }

    private IEnumerator DriveToTarget(Vector3 target)
    {
        Debug.Log($"Driving to target: {target}");
        while (true)
        {
            Vector3 dir = (target - transform.position);
            float distance = dir.magnitude;
            Debug.Log($"Distance to target: {distance}, dir: {dir}, Transform.position: {transform.position}");
            if (distance < stopThreshold) break;

            // Speed factor: 1 at far, <1 when close
            float speedFactor = Mathf.Clamp01(distance / slowDownRadius);
            float torque = maxTorque * speedFactor;

            Debug.Log($"Driving: Distance = {distance}, Speed Factor = {speedFactor}, Torque = {torque}");
            // Both sides same torque for straight drive
            // rb.AddForce(transform.forward * torque, ForceMode.Acceleration);
            // SetWheelTorque(torque, torque);
            transform.position = Vector3.MoveTowards(transform.position, target, maxSpeed * Time.deltaTime);
            yield return new WaitForFixedUpdate();
        }
        Debug.Log("Target reached. Applying brake.");
        ApplyBrake();
        yield return new WaitForSeconds(0.1f);
    }

    private void SetWheelTorque(float leftTorque, float rightTorque)
    {
        Debug.Log($"Setting wheel torque: Left = {leftTorque}, Right = {rightTorque}");
        foreach (var w in wheels)
        {
            if (w.leftWheel) w.WheelCollider.motorTorque = leftTorque;
            else if (w.rightWheel) w.WheelCollider.motorTorque = rightTorque;
        }
    }

    private void ApplyBrake()
    {
        Debug.Log("Applying brakes.");
        foreach (var w in wheels)
        {
            w.WheelCollider.motorTorque = 0f;
            w.WheelCollider.brakeTorque = maxTorque * 2f;
        }
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
    }
}