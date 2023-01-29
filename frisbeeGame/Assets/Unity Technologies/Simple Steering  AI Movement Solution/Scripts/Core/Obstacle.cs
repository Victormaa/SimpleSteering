using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace SimpleSteering { 
public class Obstacle : BaseGameEntity 
{
    /// there we only need GameObject instead of the class
    private GameObject obstacleAgent;
    public GameObject GetObstacleAgent() { return obstacleAgent; }

    //public Obstacle(float x, float y, float radius) : base(1, new Vector2(x, y), radius, 1){ }
    //public Obstacle(Vector2 pos, float radius) : base(1, pos, radius, 1) { }
    public Obstacle(int entityType, Vector2 pos, float radius, GameObject agent) 
        : base(entityType, pos, radius, 1) 
    { obstacleAgent = agent; }
    public Obstacle() : base() { }
    public void ObstacleUpdate(float time_elapsed)
    {

    }
    public void InitializedObstacle(int entityType, GameWorldPrm worldprm, ObstacleAgent<ObstacleComponent> agent, AgentManager agentManager, int spawnID = -1)
    {
        Vector2 pos;
        
        pos = new Vector2(agent.transform.position.x, agent.transform.position.z);

        SetBRadius(agent.m_radius);

        agent.InitializedObstacle(entityType, pos, BRadius());
    }
    public void InitializedObstacle_GWP(int entityType, GameWorldPrm worldprm, ObstacleAgent<ObstacleComponent> agent, AgentManager agentManager, int spawnID = -1)
    {
        var pos = new Vector2(Random.Range(worldprm.m_OxRange_min, worldprm.m_OxRange_max), Random.Range(worldprm.m_OyRange_min, worldprm.m_OyRange_max));
        SetBRadius(Random.Range(worldprm.MinObstacleRadius, worldprm.MaxObstacleRadius));

        agent.m_radius = BRadius();
        agent.transform.position = new Vector3(pos.x, 0, pos.y);
        agent.transform.rotation = Quaternion.identity;
        agent.transform.localScale = new Vector3(BRadius() * agent.radiusToScaleFactor, 1, BRadius() * agent.radiusToScaleFactor);

        agent.InitializedObstacle(entityType, pos, BRadius());
    }
}
}
