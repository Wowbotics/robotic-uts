using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

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

    public Vector3 GetWorldPos(float x, float z, float altitude)
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

    public Vector3 GetCenter(float altitude)
    {
        // Average of corners
        return new Vector3(
            (bottomLeft.x + bottomRight.x + topLeft.x + topRight.x) / 4f,
            altitude,
            (bottomLeft.y + bottomRight.y + topLeft.y + topRight.y) / 4f
        );
    }

    public float getWidth() 
    {
        return this.width; 
    }
}