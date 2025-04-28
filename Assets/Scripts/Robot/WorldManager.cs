using UnityEngine;

public class WorldManager : MonoBehaviour
{
    private int[,] grid;

    private int initial_x = 44; 
    private int initia_y = -30; 

    void Start()
    {
        // (x,y)
        // kanan bawah: (100, -85)
        // kiri atas: (-28, 25)

        // mapping world to grid: (-30, -86) => (0,0)
        int width = 135; 
        int height = 115;

        grid = new int[width, height];

        // Semua grid default 0 (belum discan)
        for (int x = 0; x < width; x++)
            for (int y = 0; y < height; y++)
                grid[x, y] = 0;
        
        // TODO: Secara manual, tandai area bounding box environment sebagai obstacle 
        
    }

    private Vector2Int WorldToGrid(Vector2 worldPos)
    {
        // Konversi posisi dunia ke grid
        int x = Mathf.FloorToInt(worldPos.x);
        int y = Mathf.FloorToInt(worldPos.y);
        x += 30; 
        y += 86; 
        return new Vector2Int(x, y);
    }

    private Vector2Int GridToWorld(Vector2Int gridPos)
    {
        // Konversi posisi grid ke dunia
        int x = gridPos.x - 30; 
        int y = gridPos.y - 86; 
        return new Vector2Int(x, y);
    }

    public int GetGridValue(Vector2 worldPos)
    {
        if (!IsInsideGrid(worldPos)) {
            // Debug.LogWarning("GetGridValue position out of bounds: " + worldPos);
            return -1; // or some other error value
        }
        Vector2Int gridPos = WorldToGrid(worldPos);
        return grid[gridPos.x, gridPos.y];
    }

    public void UpdateGrid(Vector2 worldPos, int value)
    {
        if (!IsInsideGrid(worldPos)) {
            // Debug.LogWarning("UpdateGrid position out of bounds: " + worldPos);
            return;
        }
        Vector2Int gridPos = WorldToGrid(worldPos);
        grid[gridPos.x, gridPos.y] = value;
    }

    public bool IsInsideGrid(Vector2 worldPos)
    {
        Vector2Int gridPos = WorldToGrid(worldPos);
        return gridPos.x >= 0 && gridPos.x < grid.GetLength(0) && gridPos.y >= 0 && gridPos.y < grid.GetLength(1);
    }

    public int[,] GetGrid()
    {
        return grid;
    }

    public Vector2Int? FindNearestUnscannedCell(Vector2Int worldPos)
    {
        // Debug.Log("Finding nearest unscanned cell.");
        var me = worldPos; 
        float bestDist = float.MaxValue;
        Vector2Int? best = null;
        for (int x = 0; x < GetGrid().GetLength(0); x++)
            for (int y = 0; y < GetGrid().GetLength(1); y++)
            {
                var p = new Vector2Int(x, y);
                var pWorld = GridToWorld(p); 
                
                if (GetGridValue(pWorld) == 0) // unscanned
                {
                    float d = Vector2Int.Distance(me, pWorld);
                    // Debug.Log($"Checking cell {p} coordinate {pWorld}, distance: {d}");
                    if (d < bestDist)
                    {
                        bestDist = d;
                        best = pWorld;
                        // Debug.Log($"New best cell found: {best} with distance: {bestDist}");
                    }
                }
            }
        // Debug.Log($"Nearest unscanned cell: {best}");
        return best;
    }
}