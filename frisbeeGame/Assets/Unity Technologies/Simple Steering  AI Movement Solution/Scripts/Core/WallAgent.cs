using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace SimpleSteering { 
public class WallAgent<T> : MonoBehaviour where T : MonoBehaviour
{
    private Wall2D m_Wall2D = new Wall2D();
    public Wall2D GetWall()
    {
        return m_Wall2D;
    }
    private GameObject point1;
    private GameObject point2;
    [SerializeField] private  bool UseWallForce;
    
    public void InitializedWall2D(int entityType, Vector3 from, Vector3 to)
    {
        m_Wall2D = new Wall2D(entityType, from, to, UseWallForce);
    }
    public void InitializedWall2D(int entityType)
    {
        if(transform.Find("point1") != null && transform.Find("point2") != null)
        {
            point1 = transform.Find("point1").gameObject;
            point2 = transform.Find("point2").gameObject;
        }
        else
        {
            Debug.LogError("WallAgent GameObject needs two child objects('point1' && 'point2'), two locate the position");
        }
        Vector2 from = new Vector2(point1.transform.position.x, point1.transform.position.z);
        Vector2 to = new Vector2(point2.transform.position.x, point2.transform.position.z);
        m_Wall2D = new Wall2D(entityType, from, to, UseWallForce);
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.yellow;

        var point11 = transform.Find("point1").gameObject.transform.position;
        var point22 = transform.Find("point2").gameObject.transform.position;

        Gizmos.DrawLine(point11, point22);
    }
}
}