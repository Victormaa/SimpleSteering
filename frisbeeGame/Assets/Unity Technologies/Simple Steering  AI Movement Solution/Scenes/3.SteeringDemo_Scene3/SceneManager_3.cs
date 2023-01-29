using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace SimpleSteering { 
public class SceneManager_3 : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        foreach (var vehicle in AgentManager.Instance.Vehicles())
        {
            vehicle.Steering().WallAvoidanceOn();
            vehicle.Steering().ObstacleAvoidanceOn();
            if (vehicle.GetEntityType() == (int)EntityType.AIVehicle)
            {
                vehicle.Steering().PursuitOn(AgentManager.Instance.GetRole().GetVehicle());
            }
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
}