using UnityEngine;

public class WorldManager : MonoBehaviour
{
    private int[,] grid;

    void Start()
    {
        // Ubah sesuai ukuran map
        int width = 100;
        int height = 100;
        grid = new int[width, height];

        // Semua grid default 0 (belum discan)
        for (int x = 0; x < width; x++)
            for (int y = 0; y < height; y++)
                grid[x, y] = 0;
    }

    public int GetGridValue(Vector2Int pos)
    {
        return grid[pos.x, pos.y];
    }

    public void UpdateGrid(Vector2Int pos, int value)
    {
        grid[pos.x, pos.y] = value;
    }

    public bool IsInsideGrid(Vector2Int pos)
    {
        return pos.x >= 0 && pos.x < grid.GetLength(0) && pos.y >= 0 && pos.y < grid.GetLength(1);
    }

    public int[,] GetGrid()
    {
        return grid;
    }
}


