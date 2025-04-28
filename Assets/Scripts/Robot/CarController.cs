using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

[RequireComponent(typeof(Rigidbody))]
public class CarController : MonoBehaviour {
    [Header("Drive Parameters")]
    public float maxTorque = 50f, maxSpeed = 5f, turnTorque = 150f, slowDownRadius = 1f, stopThreshold = 0.2f, turnThreshold = 0.5f;
    [Header("Waypoints")]
    public List<Vector3> waypoints = new();

    private WorldManager worldManager;
    private Queue<Vector2Int> bombQueue = new();
    private List<Vector2Int> currentPath = new();
    private Vector2Int? currentTarget;
    private bool isMoving, isDefusing;
    [SerializeField] private List<Vector2Int> boundingBoxTargets = new();
    private List<Vector2Int> remainingTargets = new();
    private Vector2Int startPosition, currentPosition;
    [SerializeField] private float lidarScanInterval = 1.0f;
    private float lidarScanTimer;
    private Rigidbody rb;
    private List<WheelControl> wheels;

    void Awake() {
        rb = GetComponent<Rigidbody>();
        wheels = new(GetComponentsInChildren<WheelControl>());
    }

    void Start() {
        worldManager = FindObjectOfType<WorldManager>();
        startPosition = currentPosition = WorldPositionToGrid(transform.position);
        if (boundingBoxTargets.Count == 0)
            boundingBoxTargets.AddRange(new[] { new Vector2Int(10, 10), new Vector2Int(20, 15), new Vector2Int(15, 25), new Vector2Int(5, 20) });
        remainingTargets = new(boundingBoxTargets);
        currentTarget = FindNearestTarget(currentPosition, remainingTargets);
        currentPath = FindPath(currentPosition, currentTarget.Value);
        if (currentPath?.Count > 0) {
            Debug.Log($"[CarController] Starting navigation to initial target {currentTarget.Value} with path length {currentPath.Count}");
            StartCoroutine(MoveAlongPath());
        }
    }

    public bool detectObstacle() {
        return Physics.Raycast(transform.position, transform.forward, out _, 3f);
    }

    void Update() {
        if (isMoving || isDefusing) return;
        OdometryUpdate();
        lidarScanTimer += Time.deltaTime;
        if (lidarScanTimer >= lidarScanInterval) { LidarScan(); lidarScanTimer = 0f; }
        if (currentTarget == null) {
            if (bombQueue.Count > 0) currentTarget = bombQueue.Peek();
            else if (remainingTargets.Count > 0) currentTarget = FindNearestTarget(currentPosition, remainingTargets);
            else {
                Debug.Log("[CarController] All targets completed.");
                return;
            }
            currentPath = FindPath(currentPosition, currentTarget.Value);
            if (currentPath == null || currentPath.Count == 0) {
                Debug.LogWarning($"[CarController] No path found to target {currentTarget.Value}, removing target.");
                if (bombQueue.Count > 0 && currentTarget == bombQueue.Peek()) bombQueue.Dequeue();
                else remainingTargets.Remove(currentTarget.Value);
                currentTarget = null;
                return;
            }
            Debug.Log($"[CarController] New path found to target {currentTarget.Value} with path length {currentPath.Count}");
        }
        if (currentPath?.Count > 0) StartCoroutine(MoveAlongPath());
    }

    public IEnumerator NavigateToWaypoints(List<Vector3> worldWaypoints) {
        foreach (var wp in worldWaypoints) {
            Vector2 wp2D = new(wp.x, wp.z);
            yield return StartCoroutine(RotateToTarget(wp2D));
            yield return StartCoroutine(DriveToTarget(wp2D));
        }
        ApplyBrake();
    }

    private Vector2 getPosition() => new(transform.position.x, transform.position.z);
    private Vector2 getForward() => new(transform.forward.x, transform.forward.z);

    private IEnumerator RotateToTarget(Vector2 target) {
        while (true) {
            Vector2 dirToTarget = target - getPosition();
            float distance = dirToTarget.magnitude;
            float desiredAngle = Vector2.SignedAngle(getForward(), dirToTarget);
            if (Mathf.Abs(desiredAngle) < 1f || distance < turnThreshold) break;
            float sign = Mathf.Sign(desiredAngle);
            SetWheelTorque(-sign * turnTorque, sign * turnTorque);
            yield return new WaitForFixedUpdate();
        }
        ApplyBrake();
        yield return new WaitForSeconds(0.1f);
    }

    private IEnumerator DriveToTarget(Vector2 target) {
        while (true) {
            Vector2 dir = target - getPosition();
            float distance = dir.magnitude;
            if (distance < stopThreshold) break;
            float speedFactor = Mathf.Clamp01(distance / slowDownRadius);
            SetWheelTorque(maxTorque * speedFactor, maxTorque * speedFactor, true);
            yield return new WaitForFixedUpdate();
        }
        ApplyBrake();
        yield return new WaitForSeconds(0.1f);
    }

    public void Move(Vector3 direction) {
        float speed = 1.0f;
        transform.position = Vector3.Lerp(transform.position, transform.position + direction * speed * Time.deltaTime, 0.5f);
    }

    private void SetWheelTorque(float leftTorque, float rightTorque, bool movingForward = false) {
        foreach (var w in wheels) {
            if (w.leftWheel) w.WheelCollider.motorTorque = leftTorque;
            else if (w.rightWheel) w.WheelCollider.motorTorque = rightTorque;
            if (movingForward) w.WheelCollider.brakeTorque = 0f;
        }
    }

    private void ApplyBrake() {
        foreach (var w in wheels) {
            w.WheelCollider.motorTorque = 0f;
            w.WheelCollider.brakeTorque = maxTorque * 10f;
        }
    }

    public IEnumerator rotate(float angle) {
        float sign = Mathf.Sign(angle);
        float targetAngle = transform.eulerAngles.y + angle;
        while (true) {
            float angleDifference = Mathf.DeltaAngle(transform.eulerAngles.y, targetAngle);
            if (Mathf.Abs(angleDifference) < 1f) break;
            SetWheelTorque(sign * turnTorque, -sign * turnTorque);
            yield return new WaitForFixedUpdate();
        }
        ApplyBrake();
        yield return new WaitForSeconds(0.1f);
    }

    public void steer(float angle) => SetWheelTorque(Mathf.Sign(angle) * turnTorque, -Mathf.Sign(angle) * turnTorque);
    public void brake(float brakeForce) => rb.velocity = Vector3.Lerp(rb.velocity, Vector3.zero, brakeForce * Time.deltaTime);
    public void drive() => SetWheelTorque(maxTorque, maxTorque, true);
    public void driveBackward() => SetWheelTorque(-maxTorque, -maxTorque, true);

    private void OdometryUpdate() {
        Vector2Int newPosition = WorldPositionToGrid(transform.position);
        if (newPosition != currentPosition) currentPosition = newPosition;
    }

    private void LidarScan() {
        Vector2Int[] directions = {
            new Vector2Int(0, 1),
            new Vector2Int(1, 0),
            new Vector2Int(0, -1),
            new Vector2Int(-1, 0)
        };
        int obstacleLayer = LayerMask.NameToLayer("Obstacle"), bombLayer = LayerMask.NameToLayer("Bomb");
        int layerMask = (1 << obstacleLayer) | (1 << bombLayer);
        for (int i = 0; i < directions.Length; i++) {
            Vector2Int dir = directions[i], scanPos = currentPosition + dir;
            Vector3 worldDir = GridToWorldDirection(dir);
            if (worldManager.IsInsideGrid(scanPos)) {
                int cell = worldManager.GetGridValue(scanPos);
                if (cell == 0) {
                    if (Physics.Raycast(transform.position, worldDir, out RaycastHit hit, 1f, layerMask)) {
                        if (hit.collider.CompareTag("Bomb")) {
                            Debug.Log($"[CarController] Bomb detected at {scanPos}");
                            worldManager.UpdateGrid(scanPos, 3);
                            bombQueue.Enqueue(scanPos);
                        } else {
                            Debug.Log($"[CarController] Obstacle detected at {scanPos}");
                            worldManager.UpdateGrid(scanPos, 2);
                        }
                    } else {
                        Debug.Log($"[CarController] Free space detected at {scanPos}");
                        worldManager.UpdateGrid(scanPos, 1);
                    }
                }
            }
        }
        DumpGridMap();
    }

    private void DumpGridMap() {
        int size = 5;
        string map = "";
        for (int y = currentPosition.y + size; y >= currentPosition.y - size; y--) {
            string line = "";
            for (int x = currentPosition.x - size; x <= currentPosition.x + size; x++) {
                Vector2Int pos = new Vector2Int(x, y);
                if (pos == currentPosition) line += "R ";
                else if (worldManager.IsInsideGrid(pos)) {
                    int val = worldManager.GetGridValue(pos);
                    line += val switch { 0 => "? ", 1 => ". ", 2 => "# ", 3 => "B ", _ => val + " " };
                } else line += "X ";
            }
            map += line + "\n";
        }
        Debug.Log(map);
    }

    private IEnumerator MoveAlongPath() {
        isMoving = true;
        foreach (var waypoint in currentPath) {
            Vector3 worldWaypoint = new Vector3(waypoint.x, transform.position.y, waypoint.y);

            // 1. Rotate to face the waypoint
            Debug.Log($"[CarController] Rotating to face waypoint {waypoint}");
            while (true) {
                Vector3 direction = (worldWaypoint - transform.position);
                direction.y = 0;
                if (direction.magnitude < 0.01f) break;
                Quaternion targetRotation = Quaternion.LookRotation(direction, Vector3.up);
                float angle = Quaternion.Angle(transform.rotation, targetRotation);
                if (angle < 3f) break;
                // Rotate in place using wheels
                float signedAngle = Vector3.SignedAngle(transform.forward, direction, Vector3.up);
                float sign = Mathf.Sign(signedAngle);
                SetWheelTorque(-sign * turnTorque, sign * turnTorque);
                yield return new WaitForFixedUpdate();
            }
            ApplyBrake();
            yield return new WaitForSeconds(0.05f);

            // 2. Move forward toward the waypoint
            Debug.Log($"[CarController] Angle to waypoint before moving: {Vector3.Angle(transform.forward, worldWaypoint - transform.position)}");
            Debug.Log($"[CarController] Moving forward to waypoint {waypoint}");
            bool stuck = false;
            float stuckTimer = 0f;
            while (Vector3.Distance(transform.position, worldWaypoint) > 0.2f) {
                Vector3 direction = (worldWaypoint - transform.position);
                direction.y = 0;
                float distance = direction.magnitude;
                float angle = Vector3.Angle(transform.forward, direction);
                if (angle > 10f) {
                    Debug.LogWarning($"[CarController] Angle to waypoint too large ({angle}), re-orienting.");
                    stuck = true;
                    break;
                }
                float speedFactor = Mathf.Clamp01(distance / slowDownRadius);
                SetWheelTorque(maxTorque * speedFactor, maxTorque * speedFactor, true);
                OdometryUpdate();
                lidarScanTimer += Time.deltaTime;
                if (lidarScanTimer >= lidarScanInterval) { LidarScan(); lidarScanTimer = 0f; }
                stuckTimer += Time.deltaTime;
                if (stuckTimer > 5f) {
                    Debug.LogWarning("[CarController] Stuck for more than 5 seconds, breaking out.");
                    stuck = true;
                    break;
                }
                yield return new WaitForFixedUpdate();
            }
            ApplyBrake();
            transform.position = worldWaypoint;
            Debug.Log($"[CarController] Reached waypoint {waypoint}");
            yield return new WaitForSeconds(0.05f);
            if (stuck) break;
        }
        if (bombQueue.Count > 0 && currentPosition == bombQueue.Peek()) StartCoroutine(DefuseBomb());
        else if (remainingTargets.Contains(currentPosition)) remainingTargets.Remove(currentPosition);
        currentPath.Clear();
        currentTarget = null;
        isMoving = false;
    }

    private IEnumerator DefuseBomb() {
        if (bombQueue.Count == 0) yield break;
        Vector2Int bombPosition = bombQueue.Dequeue();
        Debug.Log($"[CarController] Defusing bomb at {bombPosition}");
        isDefusing = true;
        yield return new WaitForSeconds(2.0f);
        worldManager.UpdateGrid(bombPosition, 1);
        Debug.Log($"[CarController] Bomb at {bombPosition} defused.");
        isDefusing = false;
    }

    private Vector2Int FindNearestTarget(Vector2Int from, List<Vector2Int> targets) =>
        targets.OrderBy(t => Vector2Int.Distance(from, t)).First();

    private Vector2Int WorldPositionToGrid(Vector3 worldPos) =>
        new Vector2Int(Mathf.RoundToInt(worldPos.x), Mathf.RoundToInt(worldPos.z));

    private Vector3 GridToWorldPosition(Vector2Int gridPos) => new Vector3(gridPos.x, 0, gridPos.y);
    private Vector3 GridToWorldDirection(Vector2Int gridDir) => new Vector3(gridDir.x, 0, gridDir.y).normalized;

    private List<Vector2Int> FindPath(Vector2Int start, Vector2Int goal) {
        var openSet = new PriorityQueue<Vector2Int>();
        var cameFrom = new Dictionary<Vector2Int, Vector2Int>();
        var gScore = new Dictionary<Vector2Int, int> { [start] = 0 };
        var fScore = new Dictionary<Vector2Int, int> { [start] = Heuristic(start, goal) };
        openSet.Enqueue(start, 0);
        while (openSet.Count > 0) {
            Vector2Int current = openSet.Dequeue();
            if (current == goal) return ReconstructPath(cameFrom, current);
            foreach (var neighbor in GetNeighbors(current)) {
                int tentativeG = gScore[current] + 1;
                if (!gScore.ContainsKey(neighbor) || tentativeG < gScore[neighbor]) {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeG;
                    fScore[neighbor] = tentativeG + Heuristic(neighbor, goal);
                    if (!openSet.Contains(neighbor)) openSet.Enqueue(neighbor, fScore[neighbor]);
                }
            }
        }
        return null;
    }

    private List<Vector2Int> GetNeighbors(Vector2Int pos) {
        Vector2Int[] directions = {
            new Vector2Int(0, 1),
            new Vector2Int(1, 0),
            new Vector2Int(0, -1),
            new Vector2Int(-1, 0)
        };
        var neighbors = new List<Vector2Int>();
        foreach (var dir in directions) {
            Vector2Int neighborGridPos = pos + dir;
            Vector3 worldPos = new Vector3(neighborGridPos.x, 0, neighborGridPos.y);
            if (worldManager.IsInsideGrid(worldPos)) {
                int cell = worldManager.GetGridValue(worldPos);
                if (cell == 1 || cell == 3 || cell == 0) neighbors.Add(neighborGridPos);
            }
        }
        return neighbors;
    }

    private int Heuristic(Vector2Int a, Vector2Int b) => Mathf.Abs(a.x - b.x) + Mathf.Abs(a.y - b.y);

    private List<Vector2Int> ReconstructPath(Dictionary<Vector2Int, Vector2Int> cameFrom, Vector2Int current) {
        var path = new List<Vector2Int> { current };
        while (cameFrom.ContainsKey(current)) {
            current = cameFrom[current];
            path.Insert(0, current);
        }
        return path;
    }

    private class PriorityQueue<T> {
        private List<(T item, int priority)> elements = new();
        public int Count => elements.Count;
        public void Enqueue(T item, int priority) => elements.Add((item, priority));
        public T Dequeue() {
            int bestIndex = 0;
            for (int i = 1; i < elements.Count; i++)
                if (elements[i].priority < elements[bestIndex].priority)
                    bestIndex = i;
            T bestItem = elements[bestIndex].item;
            elements.RemoveAt(bestIndex);
            return bestItem;
        }
        public bool Contains(T item) => elements.Any(e => EqualityComparer<T>.Default.Equals(e.item, item));
    }
}