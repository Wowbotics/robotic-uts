using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RectangleSpiralDroneController : MonoBehaviour
{
    private Rigidbody rb;

    [Header("Flight Settings")]
    [SerializeField] private float targetAltitude = 20f;
    [SerializeField] private float throttlePower = 80f;
    [SerializeField] private float moveSpeed = 30f;
    [SerializeField] private float altitudeTolerance = 0.2f;

    [Header("Search Areas")]
    [SerializeField] private List<SearchBox> searchBoxes; // List of boxes to search
    private int currentBoxIndex = 0; // Which box are we searching

    private SearchBox currentBox;
    private Vector3 currentTarget;

    [Header("Bomb Detection")]
    [SerializeField] private float detectionRadius = 0.5f;
    [SerializeField] private float detectionDistanceBomb = 17.5f;
    [SerializeField] private float detectionDistanceObstacle = 16.5f;

    private WorldManager worldManager;

    private enum State { TakeOff, MoveToCorner, Search, Finished }
    private State currentState = State.TakeOff;

    private Vector2Int currentGrid;
    private bool movingUp = true;
    private bool bombFoundInCurrentBox = false;
    private bool finished = false;

    private float gridUnit = 1f;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null) Debug.LogError("Rigidbody missing!");

        worldManager = FindObjectOfType<WorldManager>();
        if (worldManager == null)
        {
            Debug.LogError("WorldManager not found!");
        }

        if (searchBoxes.Count == 0)
        {
            Debug.LogError("No search boxes assigned!");
            return;
        }

        currentBox = searchBoxes[currentBoxIndex];
        currentGrid = Vector2Int.zero;

        currentTarget = transform.position + Vector3.up * targetAltitude; // Takeoff first
    }

    void FixedUpdate()
    {
        if (finished) return;

        Vector3 currentPos = transform.position;
        Vector3 horizontalTarget = new Vector3(currentTarget.x, currentPos.y, currentTarget.z);

        float altitudeError = targetAltitude - transform.position.y;
        float verticalVelocity = rb.velocity.y;

        float kP = 100f;
        float kD = 15f;
        float verticalForce = altitudeError * kP - verticalVelocity * kD;
        rb.AddForce(Vector3.up * verticalForce);

        float horizontalDistance = Vector3.Distance(new Vector3(currentTarget.x, 0, currentTarget.z), new Vector3(currentPos.x, 0, currentPos.z));

        Debug.Log($"Current State: {currentState}, Target: {currentTarget}, Position: {currentPos}, Altitude Error: {altitudeError}, Horizontal Distance: {horizontalDistance}");

        switch (currentState)
        {
            case State.TakeOff:
                if (Mathf.Abs(altitudeError) < altitudeTolerance)
                {
                    currentTarget = currentBox.GetWorldPos(0, 0, targetAltitude);
                    currentState = State.MoveToCorner;
                }
                break;

            case State.MoveToCorner:
                if (horizontalDistance < 0.3f)
                {
                    rb.velocity = Vector3.zero;
                    currentGrid = Vector2Int.zero;
                    currentTarget = currentBox.GetWorldPos(currentGrid.x, currentGrid.y, targetAltitude);
                    currentState = State.Search;
                }
                else
                {
                    Vector3 horizontalError = horizontalTarget - currentPos;
                    Vector3 horizontalVelocity = new Vector3(rb.velocity.x, 0, rb.velocity.z);

                    float kPh = 40f;  // Horizontal proportional gain
                    float kDh = 10f;   // Horizontal damping gain

                    Vector3 horizontalForce = (horizontalError * kPh) - (horizontalVelocity * kDh);
                    rb.AddForce(horizontalForce);
                }
                break;

            case State.Search:
                if (horizontalDistance < 0.3f)
                {
                    NextStep();
                    currentTarget = currentBox.GetWorldPos(currentGrid.x, currentGrid.y, targetAltitude);
                }
                else
                {
                    PerformMapping();
                    Vector3 error = horizontalTarget - currentPos;
                    Vector3 velocity = new Vector3(rb.velocity.x, 0, rb.velocity.z);
                    float kPh = 100f;
                    float kDh = 15f;
                    Vector3 force = (error * kPh) - (velocity * kDh);
                    rb.AddForce(force);
                }
                break;

            case State.Finished:
                rb.velocity = Vector3.zero;
                break;
        }
    }

    void PerformMapping()
    {
        RaycastHit bombHit;
        RaycastHit obstacleHit;
        Vector3 origin = transform.position;
        Vector3 direction = Vector3.down;

        Vector2Int gridPos = new Vector2Int(Mathf.RoundToInt(origin.x+18), Mathf.RoundToInt(34-origin.z));
        Debug.Log("Performing mapping at grid position: " + gridPos);

        if (worldManager.IsInsideGrid(gridPos))
        {    
            if (Physics.SphereCast(origin, detectionRadius, direction, out obstacleHit, detectionDistanceObstacle))
            {
                if (obstacleHit.collider.CompareTag("Bombs"))
                {
                    worldManager.UpdateGrid(gridPos, 3);
                } else {
                    worldManager.UpdateGrid(gridPos, 2);
                }
            } else {
                // No bomb or obstacle detected
                worldManager.UpdateGrid(gridPos, 1);
            }

            if (Physics.SphereCast(origin, detectionRadius, direction, out bombHit, detectionDistanceBomb))
            {
                if (bombHit.collider.CompareTag("Bombs"))
                {
                    // Bomb detected
                    Debug.Log("Bomb detected at: " + bombHit.collider.transform.position);
                    worldManager.UpdateGrid(gridPos, 3);
                    bombFoundInCurrentBox = true;
                    MoveToNextBox();
                }
            }
        }
        else
        {
            Debug.Log("Tried to update outside the grid!");
        }
        Debug.Log("Grid updated: " + worldManager.GetGridValue(gridPos));
        
    }

    void MoveToNextBox()
    {
        currentBoxIndex++;
        if (currentBoxIndex > searchBoxes.Count)
        {
            Debug.Log("current box index:" + currentBoxIndex);
            Debug.Log("jumlah search box: " + searchBoxes.Count);
            Debug.Log("All boxes searched! Mission complete!");
            currentState = State.Finished;
            finished = true;
            return;
        }

        Debug.Log($"Moving to next search box {currentBoxIndex + 1}");

        currentBox = searchBoxes[currentBoxIndex];
        bombFoundInCurrentBox = false;

        currentGrid = Vector2Int.zero;
        movingUp = true;

        currentTarget = currentBox.GetWorldPos(0, 0, targetAltitude);
        currentState = State.MoveToCorner;
    }

    void NextStep()
    {
        if (movingUp)
        {
            if (currentGrid.y < currentBox.zSteps - 1)
            {
                currentGrid.y++;
            }
            else
            {
                if (currentGrid.x < currentBox.xSteps - 1)
                {
                    currentGrid.x++;
                    movingUp = false;
                }
            }
        }
        else
        {
            if (currentGrid.y > 0)
            {
                currentGrid.y--;
            }
            else
            {
                if (currentGrid.x < currentBox.xSteps - 1)
                {
                    currentGrid.x++;
                    movingUp = true;
                }
            }
        }
    }

    void OnDrawGizmos()
    {
        if (searchBoxes == null) return;
        Gizmos.color = Color.cyan;

        foreach (var box in searchBoxes)
        {
            box.DrawGizmos(targetAltitude);
        }

        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(transform.position + Vector3.down * detectionDistanceBomb, detectionRadius);

        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(transform.position + Vector3.down * detectionDistanceObstacle, detectionRadius);
    }
}

[System.Serializable]
public class SearchBox
{
    public Vector2 bottomLeft;
    public Vector2 bottomRight;
    public Vector2 topLeft;
    public Vector2 topRight;

    private Vector3 origin3D => new Vector3(bottomLeft.x, 0, bottomLeft.y);
    private Vector3 xDir => new Vector3(bottomRight.x - bottomLeft.x, 0, bottomRight.y - bottomLeft.y).normalized;
    private Vector3 zDir => new Vector3(topLeft.x - bottomLeft.x, 0, topLeft.y - bottomLeft.y).normalized;

    private float width => Vector2.Distance(bottomLeft, bottomRight);
    private float height => Vector2.Distance(bottomLeft, topLeft);

    public int xSteps => Mathf.FloorToInt(width);
    public int zSteps => Mathf.FloorToInt(height);

    public Vector3 GetWorldPos(int x, int z, float altitude)
    {
        return origin3D + x * xDir + z * zDir + Vector3.up * altitude;
    }

    public void DrawGizmos(float altitude)
    {
        Vector3 bl = new Vector3(bottomLeft.x, altitude, bottomLeft.y);
        Vector3 br = new Vector3(bottomRight.x, altitude, bottomRight.y);
        Vector3 tl = new Vector3(topLeft.x, altitude, topLeft.y);
        Vector3 tr = new Vector3(topRight.x, altitude, topRight.y);

        Gizmos.DrawLine(bl, br);
        Gizmos.DrawLine(br, tr);
        Gizmos.DrawLine(tr, tl);
        Gizmos.DrawLine(tl, bl);
    }
}
