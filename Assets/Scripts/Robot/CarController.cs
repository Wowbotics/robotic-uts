using System.Collections;  
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class CarController : MonoBehaviour
{
    private WorldManager worldManager;
    private Queue<Vector2Int> bombQueue = new Queue<Vector2Int>();
    private List<Vector2Int> currentPath = new List<Vector2Int>();
    private Vector2Int? currentTarget = null;

    private CarKinematics carKine;

    private List<Vector2Int> path;


    private Queue<SearchBox> boxQueue;
     private bool isMoving = false;
    private SearchBox currentBox;


    private List<Vector3> searchWaypoints = new List<Vector3>();
    private int currentWaypointIndex = 0;
    private int currentIdx = 0;


    public float sweepSpacing = 2.0f;  // how far between sweep lines (adjustable)
    public Vector3 targetGlobal; 

    void Start()
    {
        Debug.Log("CarController started.");
        worldManager = FindObjectOfType<WorldManager>();
        carKine = GetComponent<CarKinematics>();
        boxQueue = new Queue<SearchBox>(worldManager.boxes); 
    }

    public bool detectObstacle()
    {
        RaycastHit hit;
        if (Physics.Raycast(transform.position, transform.forward, out hit, 3f))
        {
            Debug.Log("Obstacle detected.");
            return true; // Ada sesuatu di depan
        }
        else
        {
            Debug.Log("No obstacle detected.");
            return false; // Tidak ada rintangan
        }
    }

    void Update()
    {
        if (isMoving) return; 
        if (boxQueue.Count == 0) return; 
        
        currentBox = boxQueue.Dequeue(); 
        Vector3 targetPos = currentBox.GetCenter(transform.position.y); // Drive to box center first
        targetGlobal = targetPos; 
        StartCoroutine(DriveToBoxCenter());
        // carKine.MoveForward();  
    }

    private IEnumerator DriveToBoxCenter()
    {
        isMoving = true;
        // Debug.Log($"Car is moving to box center: {target}"); 
        path = FindPath(WorldPositionToGrid(transform.position), WorldPositionToGrid(targetGlobal)); 
        while (true)
        {   
            if (path == null) {
                yield return new WaitForFixedUpdate();
                continue; 
            }
            if (currentIdx >= path.Count) break;
            Vector2Int target2 = path[currentIdx];
            Vector3 toTarget = GridToWorldPosition(target2) - transform.position;
            toTarget.y = 0;
            float distance = toTarget.magnitude;

            Debug.Log($"Origin: {transform.position}, Target: {target2}, Gap : {toTarget}"); 

            // Debug.Log($"Distance to target: {distance}");
            SenseForObstacles(); 

            if (distance < 1.1f)
            {
                Debug.Log("Reached box center. Applying brake.");
                carKine.ApplyBrake();
                currentIdx++;
                yield return new WaitForFixedUpdate();
            }

            // if (Physics.Raycast(transform.position + Vector3.up * 0.5f, transform.forward, out RaycastHit hit, 2f))
            // {
            //     Debug.LogWarning("Obstacle detected in front! Stopping before search.");
            //     carKine.ApplyBrake();
            //     // rotate left or right 
            //     carKine.RotateInPlace(90); 
            //     yield break;
            // } else {
            //     carKine.MoveForward(); 
            // }

            float angle = Vector3.SignedAngle(transform.forward, toTarget.normalized, Vector3.up);
            // Debug.Log($"Angle to target: {angle}");

            if (Mathf.Abs(angle) > 5f)
            {
                float rotateDirection = Mathf.Sign(angle);
                // Debug.Log($"Rotating in place. Direction: {rotateDirection}");
                carKine.RotateInPlace(rotateDirection);
                // carKine.StopForward();
            }          
            else
            {
                // Debug.Log("Moving forward towards target.");
                carKine.MoveForward();
            }

            yield return new WaitForFixedUpdate();
        }

        Debug.Log("Reached box center. Generating search waypoints.");
        GenerateSearchWaypoints(currentBox);
        Debug.Log("Starting search inside box.");
        StartCoroutine(SearchInsideBox());
    }

    void SenseForObstacles()
    {
        float sensingDistance = 5f; // How far to "see"

        Vector3[] directions = {
            transform.forward,
            transform.right,
            -transform.right,
            Quaternion.AngleAxis(10, Vector3.up) * transform.forward,
            Quaternion.AngleAxis(-10, Vector3.up) * transform.forward,
            Quaternion.AngleAxis(30, Vector3.up) * transform.forward,
            Quaternion.AngleAxis(-30, Vector3.up) * transform.forward,
            Quaternion.AngleAxis(60, Vector3.up) * transform.forward,
            Quaternion.AngleAxis(-60, Vector3.up) * transform.forward,
            transform.forward + transform.right,
            transform.forward - transform.right
        };

        bool obstacleFound = false; 
        foreach (var dir in directions)
        {
            Ray ray = new Ray(transform.position + Vector3.up * 0.7f, dir);
            if (Physics.Raycast(ray, out RaycastHit hit, sensingDistance))
            {
                Debug.Log($"Maybe obstacle at {worldManager.WorldToGrid(WorldPositionToGrid(hit.point))} ---- {worldManager.GetGridValue(WorldPositionToGrid(hit.point))}"); 
                if (worldManager.GetGridValue(WorldPositionToGrid(hit.point)) == 0)
                { 
                    Debug.Log($"Detected new obstacle at {WorldPositionToGrid(hit.point)}");
                    worldManager.UpdateGrid(WorldPositionToGrid(hit.point), 2);
                    obstacleFound = true; 
                }
            }
        }

        if (obstacleFound) 
        {
            path = FindPath(WorldPositionToGrid(transform.position), WorldPositionToGrid(targetGlobal));
            currentIdx = 0;
            carKine.StopForward();
        }
    }

    private void GenerateSearchWaypoints(SearchBox box)
    {
        searchWaypoints.Clear();
        currentWaypointIndex = 0;

        int steps = Mathf.FloorToInt(box.getWidth() / sweepSpacing);

        for (int i = 0; i <= steps; i++)
        {
            float x = i * sweepSpacing;
            if (i % 2 == 0)
            {
                // Forward line
                searchWaypoints.Add(box.GetWorldPos(x, 0, transform.position.y));
                searchWaypoints.Add(box.GetWorldPos(x, box.zSteps, transform.position.y));
            }
            else
            {
                // Backward line
                searchWaypoints.Add(box.GetWorldPos(x, box.zSteps, transform.position.y));
                searchWaypoints.Add(box.GetWorldPos(x, 0, transform.position.y));
            }
        }
    }

    private IEnumerator SearchInsideBox()
    {
        Debug.Log("Starting search inside box...");

        while (currentWaypointIndex < searchWaypoints.Count)
        {
            Vector3 target = searchWaypoints[currentWaypointIndex];
            Vector3 toTarget = target - transform.position;
            toTarget.y = 0;

            float distance = toTarget.magnitude;

            if (distance < 1f)
            {
                currentWaypointIndex++;
                continue;
            }

            if (Physics.Raycast(transform.position + Vector3.up * 0.5f, transform.forward, out RaycastHit hit, 2f))
            {
                if (hit.collider.CompareTag("Obstacle"))
                {
                    Debug.LogWarning("Obstacle detected during search! Stopping.");
                    carKine.ApplyBrake();
                    yield break;
                }
            }

            float angle = Vector3.SignedAngle(transform.forward, toTarget.normalized, Vector3.up);
            if (Mathf.Abs(angle) > 5f)
            {
                float rotateDirection = Mathf.Sign(angle);
                carKine.RotateInPlace(rotateDirection);
                carKine.StopForward();
            }
            else
            {
                carKine.MoveForward();
            }

            yield return new WaitForFixedUpdate();
        }

        Debug.Log("Finished searching the box!");
        isMoving = false;
    }

    private void LidarScan()
    {
        Debug.Log("Performing Lidar scan.");
        Vector2Int currentPos = WorldPositionToGrid(transform.position);

        Vector2Int[] directions = {
            new Vector2Int(0, 1),
            new Vector2Int(1, 0),
            new Vector2Int(0, -1),
            new Vector2Int(-1, 0)
        };

        foreach (var dir in directions)
        {
            Vector2Int scanPos = currentPos + dir;

            // Debug.Log($"Scanning direction {dir} from position {currentPos} to {scanPos}.");
            // Debug.Log($"Is scanPos inside grid? {worldManager.IsInsideGrid(scanPos)}");

            if (worldManager.IsInsideGrid(scanPos))
            {
                int cell = worldManager.GetGridValue(scanPos);

                if (cell == 0) // Belum discan
                {
                    Vector3 worldDir = GridToWorldPosition(dir); 
                    RaycastHit hit;
                    Debug.Log($"Raycasting from {transform.position} to {scanPos} in direction {worldDir}.");
                    if (Physics.Raycast(transform.position, worldDir, out hit, 2f))
                    {
                        if (hit.collider.CompareTag("Bombs"))
                        {
                            Debug.Log($"Bomb detected at {scanPos}");
                            worldManager.UpdateGrid(scanPos, 3); // Bomb
                            bombQueue.Enqueue(scanPos);
                        }
                        else 
                        {
                            Debug.Log($"Obstacle detected at {scanPos}");
                            worldManager.UpdateGrid(scanPos, 2); // Obstacle
                        }
                    } 
                    else
                    {
                        Debug.Log($"Path detected at {scanPos}");
                        worldManager.UpdateGrid(scanPos, 1); // Jalan
                    }
                }
            }
        }
    }

    // private IEnumerator MoveAlongPath()
    // {
    //     Debug.Log("Starting movement along path.");
    //     isMoving = true;

    //     var worldWaypoints = currentPath
    //         .Select(g => new Vector3(g.x, transform.position.y, g.y))
    //         .ToList();

    //     Debug.Log($"Navigating to waypoints: {string.Join(", ", worldWaypoints)}");
    //     yield return StartCoroutine(carKine.NavigateToWaypoints(worldWaypoints));

    //     Debug.Log("Path traversal complete.");

    //     currentPath.Clear();
    //     currentTarget = null;
    //     isMoving = false;
    // }

    private Vector2Int WorldPositionToGrid(Vector3 worldPos)
    {
        return new Vector2Int(Mathf.RoundToInt(worldPos.x), Mathf.RoundToInt(worldPos.z));
    }

    private Vector3 GridToWorldPosition(Vector2Int gridPos)
    {
        return new Vector3(gridPos.x, 0, gridPos.y);
    }

    private List<Vector2Int> FindPath(Vector2Int start, Vector2Int goal)
    {
        Debug.Log($"Finding path from {start} to {goal}.");
        var openSet = new PriorityQueue<Vector2Int>();
        var cameFrom = new Dictionary<Vector2Int, Vector2Int>();
        var gScore = new Dictionary<Vector2Int, int>();
        var fScore = new Dictionary<Vector2Int, int>();

        openSet.Enqueue(start, 0);
        gScore[start] = 0;
        fScore[start] = Heuristic(start, goal);

        while (openSet.Count > 0)
        {
            Vector2Int current = openSet.Dequeue();
            Debug.Log($"Processing node {current}.");

            if (current == goal)
            {
                Debug.Log("Goal reached. Reconstructing path.");
                return ReconstructPath(cameFrom, current);
            }

            foreach (var neighbor in GetNeighbors(current))
            {
                int tentativeG = gScore[current] + 1;

                if (!gScore.ContainsKey(neighbor) || tentativeG < gScore[neighbor])
                {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeG;
                    fScore[neighbor] = tentativeG + Heuristic(neighbor, goal);

                    if (!openSet.Contains(neighbor))
                    {
                        Debug.Log($"Adding neighbor {neighbor} to open set with priority {fScore[neighbor]}.");
                        openSet.Enqueue(neighbor, fScore[neighbor]);
                    }
                }
            }
        }

        Debug.LogWarning("Path not found.");
        return null; // Path tidak ketemu
    }

    private List<Vector2Int> GetNeighbors(Vector2Int pos)
    {
        Vector2Int[] directions = {
            new Vector2Int(0,1),
            new Vector2Int(1,0),
            new Vector2Int(0,-1),
            new Vector2Int(-1,0)
        };

        List<Vector2Int> neighbors = new List<Vector2Int>();

        foreach (var dir in directions)
        {
            Vector2Int neighbor = pos + dir;

            if (worldManager.IsInsideGrid(neighbor))
            {
                int cell = worldManager.GetGridValue(neighbor);
                if (cell == 0 || cell == 1 || cell == 3) // 1 jalan biasa, 3 bom
                {
                    neighbors.Add(neighbor);
                    Debug.Log($"Neighbor {neighbor} added.");
                }
            }
        }

        return neighbors;
    }

    private int Heuristic(Vector2Int a, Vector2Int b)
    {
        return Mathf.Abs(a.x - b.x) + Mathf.Abs(a.y - b.y);
    }

    private List<Vector2Int> ReconstructPath(Dictionary<Vector2Int, Vector2Int> cameFrom, Vector2Int current)
    {
        List<Vector2Int> path = new List<Vector2Int> { current };
        while (cameFrom.ContainsKey(current))
        {
            current = cameFrom[current];
            path.Insert(0, current);
        }
        Debug.Log($"Reconstructed path: {string.Join(" -> ", path)}");
        return path;
    }

    private class PriorityQueue<T>
    {
        private List<(T item, int priority)> elements = new List<(T, int)>();

        public int Count => elements.Count;

        public void Enqueue(T item, int priority)
        {
            elements.Add((item, priority));
        }

        public T Dequeue()
        {
            int bestIndex = 0;
            for (int i = 1; i < elements.Count; i++)
                if (elements[i].priority < elements[bestIndex].priority)
                    bestIndex = i;

            T bestItem = elements[bestIndex].item;
            elements.RemoveAt(bestIndex);
            return bestItem;
        }

        public bool Contains(T item)
        {
            return elements.Any(e => EqualityComparer<T>.Default.Equals(e.item, item));
        }
    }
}
