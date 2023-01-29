using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace SimpleSteering { 
public class Wall2D : BaseGameEntity 
{
    /// there we only need GameObject instead of the class
    private GameObject wall2DAgent;
    public GameObject GetObstacleAgent() { return wall2DAgent; }
    private bool UseWallForce;
    public bool GetUseWallForce() { return UseWallForce; }
    protected Vector2 m_vA, m_vB, m_vN;
    protected void CalculateNormal()
    {
        Vector2 temp = (m_vB - m_vA).normalized;

        m_vN.x = -temp.y;
        m_vN.y = temp.x;
    }
    public Wall2D():base() { }
    public Wall2D(int entityType, Vector2 A, Vector2 B, bool usewallForce) :base(entityType)
    {
        m_vA = A;
        m_vB = B;
        CalculateNormal();
        UseWallForce = usewallForce;
    }
    public Wall2D(int entityType, Vector2 A, Vector2 B, Vector2 N) : base(entityType)
    {
        m_vA = A;
        m_vB = B;
        m_vN = N;
    }
    public Wall2D(string wallParamFile)
    {

    }
    public virtual void Render(bool renderNormals = false)
    {
        if (renderNormals)
        {
            Vector2 line = m_vA + m_vB;
            Vector2 lineMiddle = new Vector2(line.x / 2, line.y / 2);
        }
    }
    public Vector2 From() { return m_vA; }
    public void SetFrom(Vector2 from)
    {
        m_vA = from;
        CalculateNormal();
    }
    public Vector2 To() { return m_vB; }
    public void SetTo(Vector2 to)
    {
        m_vB = to;
        CalculateNormal();
    }
    public Vector2 Normal() { return m_vN; }
    public Vector2 GetWallLine() { return m_vA + m_vB; }
    public Vector2 Center() { return (m_vA + m_vB) / 2.0f; }

}
}