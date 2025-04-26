using UnityEngine;

public class ObstacleDetection : MonoBehaviour
{
    [Header("Obstacle Detection Settings")]
    public float detectionRange = 50f; // Range to detect obstacles
    public LayerMask obstacleLayer;   // Layer mask for obstacles

    public bool IsObstacleDetected()
    {
        // Perform a raycast to detect obstacles in front of the robot
        RaycastHit hit;
        if (Physics.Raycast(transform.position, transform.forward, out hit, detectionRange, obstacleLayer))
        {
            Debug.DrawLine(transform.position, hit.point, Color.red); // Debug line for visualization
            return true;
        }
        return false;
    }
}