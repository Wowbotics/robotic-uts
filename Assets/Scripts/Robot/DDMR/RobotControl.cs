using UnityEngine;

public class RobotControl : MonoBehaviour
{
    [Header("Robot Properties")]
    public float motorTorque = 700;
    public float brakeTorque = 400;
    public float maxSpeed = 4f; 
    public float centreOfGravityOffset = -1f;

    private WheelControl[] wheels;
    private Rigidbody rigidBody;

    private Vector3 targetPoint; // Current target point
    private bool isMovingToPoint = false; // Flag to indicate if the robot is moving to a point

    // Start is called before the first frame update
    void Start()
    {
        rigidBody = GetComponent<Rigidbody>();

        // Adjust center of mass to improve stability and prevent rolling
        Vector3 centerOfMass = rigidBody.centerOfMass;
        centerOfMass.y += centreOfGravityOffset;
        rigidBody.centerOfMass = centerOfMass;

        // Get all wheel components attached to the robot
        wheels = GetComponentsInChildren<WheelControl>();

        // Example: Move through a series of points
        Vector3[] points = new Vector3[]
        {
            new Vector3(55.4f, 2.16f, -29.1f),
            new Vector3(55.4f, 2.16f, -42.0f),
            new Vector3(55.4f, 2.16f, -44.0f),
            new Vector3(61.2f, 2.16f, -44.7f),
            new Vector3(55.4f, 2.16f, -35.4f)
        };
        MoveThroughPoints(points);
    }

    // Method to move the robot to a specific point
    public void MoveToPoint(Vector3 point)
    {
        targetPoint = point;
        isMovingToPoint = true;
    }

    public void MoveThroughPoints(Vector3[] points)
    {
        StartCoroutine(MoveThroughPointsCoroutine(points));
    }

    private System.Collections.IEnumerator MoveThroughPointsCoroutine(Vector3[] points)
    {
        foreach (var point in points)
        {
            MoveToPoint(point);

            // Wait until the robot reaches the current point
            while (isMovingToPoint)
            {
                yield return null;
            }
        }
    }

    // FixedUpdate is called at a fixed time interval
    void FixedUpdate()
    {
        if (isMovingToPoint)
        {
            Vector3 directionToTarget = (targetPoint - transform.position).normalized;
            float angleToTarget = Vector3.SignedAngle(transform.forward, directionToTarget, Vector3.up);
            float distanceToTarget = Vector3.Distance(transform.position, targetPoint);

            // Calculate inputs for differential drive
            float forwardInput = Mathf.Clamp(distanceToTarget / 5f, 0.5f, 1f); // Slow down as distance decreases
            float turnInput = Mathf.Clamp(angleToTarget / 30f, -1f, 1f); // Adjust turning sensitivity

            if (distanceToTarget < 0.5f)
            {
                isMovingToPoint = false;
                StopRobot();
                return;
            }

            ApplyDifferentialDrive(forwardInput, turnInput);
        }
        else
        {
            // Manual control for testing
            float forwardInput = Input.GetAxis("Vertical");
            float turnInput = Input.GetAxis("Horizontal");
            ApplyDifferentialDrive(forwardInput, turnInput);
        }
    }

    // Apply movement using differential drive logic
    private void ApplyDifferentialDrive(float forwardInput, float turnInput)
    {
        foreach (var wheel in wheels)
        {
            if (wheel == null || wheel.WheelCollider == null)
                continue;

            float motorTorqueValue = forwardInput * motorTorque;

            // Differential drive logic: Left wheels get (forward + turn), Right wheels get (forward - turn)
            if (wheel.leftWheel)
            {
                wheel.WheelCollider.motorTorque = motorTorqueValue + (turnInput * motorTorque);
            }
            else if (wheel.rightWheel)
            {
                wheel.WheelCollider.motorTorque = motorTorqueValue - (turnInput * motorTorque);
            }

            // No braking unless stopping
            wheel.WheelCollider.brakeTorque = 0f;
        }
    }

    // Helper to stop the robot
    private void StopRobot()
    {
        foreach (var wheel in wheels)
        {
            wheel.WheelCollider.motorTorque = 0f;
            wheel.WheelCollider.brakeTorque = brakeTorque;
        }
    }
}