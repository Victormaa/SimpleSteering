using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace SimpleSteering { 
public class DrawDebug : MonoBehaviour
{
    public AgentManager m_AgentManager;

    [Header("Vehicle", order = 0)]
    public bool Draw_ViewDistance;
    public bool Draw_BoundingRadius;
    public bool Draw_FacingDir;
    public bool Draw_Feelers;
    public bool Draw_CurrentForce;
    [Header("Steering", order = 1)]
    public bool Draw_WanderingTarget;
    [Header("Obstacle", order = 2)]
    public bool Draw_Obstacle;

    [Header("Circle", order = 3)]
    [HideInInspector] public float ThetaScale = 0.02f;
    private int Size;
    private float Theta = 0f;
    private void DrawCircle(Vector2 pos, float radius, LineRenderer LineDrawer)
    {
        Theta = 0f;
        Size = (int)((1f / ThetaScale) + 1f);
        LineDrawer.positionCount = Size;
        for (int i = 0; i < Size; i++)
        {
            Theta += (2.0f * Mathf.PI * ThetaScale);
            float x = radius * Mathf.Cos(Theta);
            float y = radius * Mathf.Sin(Theta);
            LineDrawer.SetPosition(i, new Vector3(x+ pos.x, 0, y + pos.y));
        }
    }
    // Update is called once per frame
    void Update()
    {
        VehicleDebugDraw();
    }

    void VehicleDebugDraw()
    {
        foreach(var vehicle in m_AgentManager.Vehicles())
        {
            var center = new Vector3(vehicle.Pos().x, 0, vehicle.Pos().y);
            Vector3 normal = Vector3.up;
            var lineRender = vehicle.Agent().GetComponent<LineRenderer>();
            /// bounding radius
            if (Draw_BoundingRadius)
            {
                
                lineRender.loop = true;
                DrawCircle(vehicle.Pos(), vehicle.BRadius(), lineRender);
            }
                
            /// view distance
            if (Draw_ViewDistance)
            {
                lineRender.loop = true;
                DrawCircle(vehicle.Pos(), vehicle.Steering().GetViewDist(), lineRender);
            }

            /// Draw facing dir
            if (Draw_FacingDir)
            {
                Debug.DrawLine(
               vehicle.Agent().transform.position,
               vehicle.Agent().transform.position + new Vector3(vehicle.Facing().x, 0, vehicle.Facing().y) * 5.0f);
            }
            if (Draw_Feelers)
            {
                var feelers = vehicle.Steering().GetFeelers();
                if (feelers != null)
                {
                    Debug.DrawLine
                        (vehicle.Agent().transform.position,
                        new Vector3(feelers[0].x, vehicle.Agent().transform.position.y, feelers[0].y));
                    Debug.DrawLine
                        (vehicle.Agent().transform.position,
                        new Vector3(feelers[1].x, vehicle.Agent().transform.position.y, feelers[1].y));
                    Debug.DrawLine
                        (vehicle.Agent().transform.position,
                        new Vector3(feelers[2].x, vehicle.Agent().transform.position.y, feelers[2].y));
                }
            }
            if (Draw_CurrentForce)
            {
                Debug.DrawLine
                        (vehicle.Agent().transform.position,
                        new Vector3(vehicle.Pos().x + vehicle.GetCurrentForce().x, vehicle.Agent().transform.position.y, vehicle.Pos().y + vehicle.GetCurrentForce().y));
            }
            if (Draw_WanderingTarget)
            {
                if(vehicle.Steering().isWanderOn())
                {
                    Debug.DrawLine(new Vector3(vehicle.Pos().x, 0, vehicle.Pos().y),
                    new Vector3((vehicle.Steering().GetCurWanderTarget()).x, 0, (vehicle.Steering().GetCurWanderTarget()).y),
                    Color.green);
                }
            }
            if(!Draw_ViewDistance && !Draw_BoundingRadius)
            {
                lineRender.positionCount = 0;
            }
        }
        foreach(var obstacle in m_AgentManager.Obstacles())
        {
            var lineRender = obstacle.GetObstacleAgent().gameObject.GetComponent<LineRenderer>();
            if (Draw_Obstacle)
            {
                lineRender.loop = true;
                DrawCircle(obstacle.Pos(), obstacle.BRadius(), lineRender);
                
            }
            else
            {
                lineRender.positionCount = 0;
            }
        }
    }
}
}
