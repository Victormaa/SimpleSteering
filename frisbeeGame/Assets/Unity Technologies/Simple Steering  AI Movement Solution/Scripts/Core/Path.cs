using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace SimpleSteering { 
public class Path 
{
    private List<Vector2> m_WayPoints;
    private List<Vector3> m_DrawWayPoints;
    private Vector2 m_CurrentWayPoint;
    private bool m_bLooped;
    private int m_CurrentWayPointID;

    public Path()
    {
        m_WayPoints = new List<Vector2>();
        m_DrawWayPoints = new List<Vector3>();
        m_CurrentWayPoint = Vector2.zero;
        m_bLooped = false;
        m_CurrentWayPointID = 0;
    }

    public Path(int NumWaypoints,
                float MinX,
                float MinY,
                float MaxX,
                float MaxY,
                bool looped) 
    {
        m_bLooped = looped;
        m_WayPoints = new List<Vector2>();
        m_DrawWayPoints = new List<Vector3>();
        CreateRandomPath(NumWaypoints, MinX, MinY, MaxX, MaxY);
        m_CurrentWayPoint = m_WayPoints[0];
        m_CurrentWayPointID = 0;
    }
    public Vector2 CurrentWayPoint() { return m_CurrentWayPoint; }
    public bool Finished() 
    {
        return !(m_CurrentWayPoint != m_WayPoints[m_WayPoints.Count - 1]);
    }
    public void SetNextWayPoint()
    {
        if(m_WayPoints.Count == 0)
        {
            Debug.LogError("there is no path created");
            return;
        }
        m_CurrentWayPointID += 1;
        int count = m_WayPoints.Count;
        if (m_CurrentWayPointID > count - 1)
        {
            if (m_bLooped)
            {
                m_CurrentWayPoint = m_WayPoints[0];
                m_CurrentWayPointID = 0;
            }
            else
            {
                m_CurrentWayPointID -= 1;
            }
        }
        else
        {
            m_CurrentWayPoint = m_WayPoints[m_CurrentWayPointID];
        }
    }
    public void LoopOn() { m_bLooped = true; }
    public void LoopOff() { m_bLooped = false; }
    public void Clear() { m_WayPoints.Clear(); }
    public List<Vector2> GetPath() { return m_WayPoints; }
    public void Set(Path path) 
    { 
        m_WayPoints = path.GetPath();
        m_CurrentWayPoint = m_WayPoints[0];
        m_CurrentWayPointID = 0;
    }
    public void Set(List<Vector2> newWayPoints)
    {
        m_WayPoints = newWayPoints;
        foreach(var waypoint in m_WayPoints)
        {
            m_DrawWayPoints.Add(new Vector3(waypoint.x, 0, waypoint.y));
        }        
        m_CurrentWayPoint = m_WayPoints[0];
        m_CurrentWayPointID = 0;
    }
    public void AddWayPoint(Vector2 newPoint)
    {
        m_WayPoints.Add(newPoint);
    }
    private void CreateRandomPath(int NumWaypoints,
                                  float MinX,
                                  float MinY,
                                  float MaxX,
                                  float MaxY)
    {
        m_WayPoints.Clear();
        m_DrawWayPoints.Clear();

        float midX = (MaxX + MinX) / 2;
        float midY = (MaxY + MinY) / 2;
        float smaller = Mathf.Min(midX, midY);

        float spacing = 360.0f / NumWaypoints; //Mathf.PI * 2.0f / NumWaypoints;

        for (int i = 0; i < NumWaypoints; ++i)
        {
            float RadialDist = Random.Range(smaller * 0.2f, smaller);

            Vector2 temp = new Vector2(RadialDist, 0.0f);

            temp = Common.Vec2DRotateAroundOrigin(temp, i * spacing);

            temp.x += midX; temp.y += midY;

            m_WayPoints.Add(temp);

            m_DrawWayPoints.Add(new Vector3(temp.x, 0, temp.y));
        }

        m_CurrentWayPoint = m_WayPoints[0];
    }

    public virtual void Render()
    {
        //Drawing.Draw.Polyline(m_DrawWayPoints, m_bLooped, Color.red);
    }
    private bool m_isrender = false;
    public void RenderOn() { m_isrender = true; }
    public void RenderOff() { m_isrender = false; }
    public bool isRender() { return m_isrender; }
}
}

