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
    private bool isMoving = false;

    void Start()
    {
        Debug.Log("CarController started.");
        worldManager = FindObjectOfType<WorldManager>();
        carKine = GetComponent<CarKinematics>();
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
        Debug.Log("Update called.");
        LidarScan();

        if (currentTarget == null)
        {
            Debug.Log("No current target. Checking bomb queue or do ");
            if (bombQueue.Count > 0)
            {
                currentTarget = bombQueue.Dequeue();
                Debug.Log($"Dequeued bomb target: {currentTarget}");
            }
            else
            {
                var next = worldManager.FindNearestUnscannedCell(WorldPositionToGrid(transform.position));
                if (next.HasValue)
                {
                    currentTarget = next.Value;
                    Debug.Log($"Nearest unscanned cell set as target: {currentTarget}");
                }
                else
                {
                    Debug.Log("No unscanned cells found. Exiting Update.");
                    return;
                }
            }
            currentPath = FindPath(WorldPositionToGrid(transform.position), currentTarget.Value);
        }

        if (currentTarget != null && currentPath != null && currentPath.Count > 0)
        {
            Debug.Log($"Path to target calculated: {string.Join(" -> ", currentPath)}");
            Debug.Log("Starting movement along path.");
            StartCoroutine(MoveAlongPath());
        }
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
                    if (Physics.Raycast(transform.position, worldDir, out hit, 1f))
                    {
                        if (hit.collider.CompareTag("Bomb"))
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

    private IEnumerator MoveAlongPath()
    {
        Debug.Log("Starting movement along path.");
        isMoving = true;

        var worldWaypoints = currentPath
            .Select(g => new Vector3(g.x, transform.position.y, g.y))
            .ToList();

        Debug.Log($"Navigating to waypoints: {string.Join(", ", worldWaypoints)}");
        yield return StartCoroutine(carKine.NavigateToWaypoints(worldWaypoints));

        Debug.Log("Path traversal complete.");

        currentPath.Clear();
        currentTarget = null;
        isMoving = false;
    }

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
                if (cell == 1 || cell == 3) // 1 jalan biasa, 3 bom
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
