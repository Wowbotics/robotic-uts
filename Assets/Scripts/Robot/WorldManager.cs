using UnityEngine;

public class WorldManager : MonoBehaviour
{
    private int[,] grid;

    private int width = 135;   // grid width
    private int height = 115;  // grid height

    // World to grid offset
    private int worldOffsetX = 30;
    private int worldOffsetY = 86;

// In WorldManager.cs
void Awake() // Changed from Start()
{
    InitializeGrid();
}

    void InitializeGrid()
    {
        grid = new int[width, height];

        // Semua grid default 0 (belum discan)
        for (int x = 0; x < width; x++)
            for (int y = 0; y < height; y++)
                grid[x, y] = 0;

        DefineBoundingBoxObstacle();
        PlaceDummyBombs(); // Add this line
    }

    void DefineBoundingBoxObstacle()
    {
        // Buat keliling arena jadi obstacle (-1)
        for (int x = 0; x < width; x++)
        {
            grid[x, 0] = -1; // bawah
            grid[x, height - 1] = -1; // atas
        }
        for (int y = 0; y < height; y++)
        {
            grid[0, y] = -1; // kiri
            grid[width - 1, y] = -1; // kanan
        }

        // Tambah obstacles tambahan jika arena kamu ada dinding dalam
        // Contoh obstacle di area tertentu (optional, tergantung layout arena kamu)
        /*
        for (int x = 50; x <= 80; x++)
            for (int y = 40; y <= 60; y++)
                grid[x, y] = -1; // obstacle kotak
        */
    }

    // Add this method to place dummy bombs
    void PlaceDummyBombs()
    {
        // Example bomb locations (in grid coordinates, adjust as needed)
        var bombPositions = new Vector2Int[]
        {
            new Vector2Int(15, 20),
            new Vector2Int(30, 40),
            new Vector2Int(50, 60)
        };

        foreach (var pos in bombPositions)
        {
            if (pos.x >= 0 && pos.x < width && pos.y >= 0 && pos.y < height)
                grid[pos.x, pos.y] = 3; // 3 = bomb
        }
    }

    private Vector2Int WorldToGrid(Vector2 worldPos)
    {
        int x = Mathf.FloorToInt(worldPos.x) + worldOffsetX;
        int y = Mathf.FloorToInt(worldPos.y) + worldOffsetY;
        return new Vector2Int(x, y);
    }

    private Vector2Int GridToWorld(Vector2Int gridPos)
    {
        int x = gridPos.x - worldOffsetX;
        int y = gridPos.y - worldOffsetY;
        return new Vector2Int(x, y);
    }

    public int GetGridValue(Vector2 worldPos)
    {
        if (!IsInsideGrid(worldPos)) return -1;
        Vector2Int gridPos = WorldToGrid(worldPos);
        return grid[gridPos.x, gridPos.y];
    }

    public void UpdateGrid(Vector2 worldPos, int value)
    {
        if (!IsInsideGrid(worldPos)) return;
        Vector2Int gridPos = WorldToGrid(worldPos);
        grid[gridPos.x, gridPos.y] = value;
    }

    public bool IsInsideGrid(Vector2 worldPos)
    {
        Vector2Int gridPos = WorldToGrid(worldPos);
        return gridPos.x >= 0 && gridPos.x < width && gridPos.y >= 0 && gridPos.y < height;
    }

    public int[,] GetGrid()
    {
        return grid;
    }

    public Vector2Int? FindNearestUnscannedCell(Vector2Int worldPos)
    {
        var me = worldPos;
        float bestDist = float.MaxValue;
        Vector2Int? best = null;

        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                if (grid[x, y] == 0) // unscanned
                {
                    Vector2Int pWorld = GridToWorld(new Vector2Int(x, y));
                    float d = Vector2Int.Distance(me, pWorld);
                    if (d < bestDist)
                    {
                        bestDist = d;
                        best = pWorld;
                    }
                }
            }
        }
        return best;
    }
}
