using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class CarController : MonoBehaviour
{
    private WorldManager worldManager;
    private Queue<Vector2Int> bombQueue = new Queue<Vector2Int>();
    private List<Vector2Int> currentPath = new List<Vector2Int>();
    private Vector2Int? currentTarget = null;

    void Start()
    {
        worldManager = FindObjectOfType<WorldManager>();
    }

    void Update()
    {
        LidarScan();

        if (currentTarget == null && bombQueue.Count > 0)
        {
            currentTarget = bombQueue.Dequeue();
            currentPath = FindPath(WorldPositionToGrid(transform.position), currentTarget.Value);
        }

        if (currentTarget != null && currentPath != null && currentPath.Count > 0)
        {
            MoveAlongPath();
        }
    }

    private void LidarScan()
    {
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

            if (worldManager.IsInsideGrid(scanPos))
            {
                int cell = worldManager.GetGridValue(scanPos);

                if (cell == 0) // Belum discan
                {
                    RaycastHit hit;
                    Vector3 worldScanPos = GridToWorldPosition(scanPos);
                    if (Physics.Raycast(worldScanPos + Vector3.up * 5, Vector3.down, out hit, 10f))
                    {
                        if (hit.collider.CompareTag("Obstacle"))
                        {
                            worldManager.UpdateGrid(scanPos, 2); // Obstacle
                        }
                        else if (hit.collider.CompareTag("Bomb"))
                        {
                            worldManager.UpdateGrid(scanPos, 3); // Bomb
                            bombQueue.Enqueue(scanPos);
                        }
                        else
                        {
                            worldManager.UpdateGrid(scanPos, 1); // Jalan
                        }
                    }
                }
            }
        }
    }

    private void MoveAlongPath()
    {
        if (currentPath == null || currentPath.Count == 0)
            return;

        Vector2Int nextGrid = currentPath[0];
        Vector3 targetWorldPos = GridToWorldPosition(nextGrid);

        float speed = 3.0f;
        transform.position = Vector3.MoveTowards(transform.position, targetWorldPos, speed * Time.deltaTime);

        if (Vector3.Distance(transform.position, targetWorldPos) < 0.1f)
        {
            currentPath.RemoveAt(0);

            if (currentPath.Count == 0)
            {
                Debug.Log("Bom ditemukan dan didefuse!");
                worldManager.UpdateGrid(nextGrid, 1); // Kosongkan bom
                currentTarget = null;
            }
        }
    }

    private Vector2Int WorldPositionToGrid(Vector3 worldPos)
    {
        return new Vector2Int(Mathf.RoundToInt(worldPos.x), Mathf.RoundToInt(worldPos.z));
    }

    private Vector3 GridToWorldPosition(Vector2Int gridPos)
    {
        return new Vector3(gridPos.x, 0, gridPos.y);
    }

    // =========================
    // A* PATHFINDING DI SINI
    // =========================
    private List<Vector2Int> FindPath(Vector2Int start, Vector2Int goal)
    {
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

            if (current == goal)
                return ReconstructPath(cameFrom, current);

            foreach (var neighbor in GetNeighbors(current))
            {
                int tentativeG = gScore[current] + 1;

                if (!gScore.ContainsKey(neighbor) || tentativeG < gScore[neighbor])
                {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeG;
                    fScore[neighbor] = tentativeG + Heuristic(neighbor, goal);

                    if (!openSet.Contains(neighbor))
                        openSet.Enqueue(neighbor, fScore[neighbor]);
                }
            }
        }

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
                    neighbors.Add(neighbor);
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


