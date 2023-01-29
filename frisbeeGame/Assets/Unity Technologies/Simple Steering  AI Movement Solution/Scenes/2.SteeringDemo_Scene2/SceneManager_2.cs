using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace SimpleSteering { 
public class SceneManager_2 : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        foreach(var vehicle in AgentManager.Instance.Vehicles())
        {
            if(vehicle.GetEntityType() == (int)EntityType.AIVehicle)
            {
                vehicle.Steering().WallAvoidanceOn();
                vehicle.Steering().ObstacleAvoidanceOn();
                vehicle.Steering().WanderOn();
                //vehicle.Steering().FlockingOn();
            }
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
}
