using System.Collections;  
using System.Collections.Generic;
using UnityEngine;
public class WorldManager : MonoBehaviour
{
    private int[,] grid;

    [Header("Boxes to Search")]
    public List<SearchBox> boxes;
    public bool[] done;
    public Vector2 [] bombLocations; 
    private int accuracy = 8;

    void Start()
    {
        // (x,y)
        // kanan bawah: (100, -85)
        // kiri atas: (-28, 25)

        // mapping world to grid: (-30, -86) => (0,0)
        int width = 135 * (accuracy+2); 
        int height = 115 * (accuracy+2);

        grid = new int[width, height];

        // Semua grid default 0 (belum discan)
        for (int x = 0; x < width; x++)
            for (int y = 0; y < height; y++)
                grid[x, y] = 0;
        
        int bombCount = boxes.Count;  
        done = new bool[bombCount]; 
        for (int i = 0; i<bombCount; i++) done[i] = false; 
    }

    public Vector2Int WorldToGrid(Vector2 worldPos)
    {
        // Konversi posisi dunia ke grid
        int x = Mathf.FloorToInt(worldPos.x*accuracy);
        int y = Mathf.FloorToInt(worldPos.y*accuracy);
        x += 30*accuracy; 
        y += 86*accuracy; 
        return new Vector2Int(x, y);
    }

    private Vector2Int GridToWorld(Vector2Int gridPos)
    {
        // Konversi posisi grid ke dunia
        int x = gridPos.x/accuracy - 30*accuracy; 
        int y = gridPos.y/accuracy - 86*accuracy; 
        return new Vector2Int(x, y);
    }

    public int GetGridValue(Vector2 worldPos)
    {
        if (!IsInsideGrid(worldPos)) {
            Debug.LogWarning("GetGridValue position out of bounds: " + worldPos);
            return -1; // or some other error value
        }
        Vector2Int gridPos = WorldToGrid(worldPos);
        return grid[gridPos.x, gridPos.y];
    }

    public void UpdateGrid(Vector2 worldPos, int value)
    {
        if (!IsInsideGrid(worldPos)) {
            Debug.LogWarning("UpdateGrid position out of bounds: " + worldPos);
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

    public void handleBombFoundDrone(Vector2 worldPos, int bombIndex) 
    { 
        bombLocations[bombIndex] = worldPos; 
        done[bombIndex] = true;
    }

    public void handleBombFoundCar(int bombIndex) 
    {
        done[bombIndex] = true; 
    }
}