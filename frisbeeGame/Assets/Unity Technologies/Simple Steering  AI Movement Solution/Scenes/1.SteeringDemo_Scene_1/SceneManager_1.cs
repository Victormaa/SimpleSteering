using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace SimpleSteering { 
public class SceneManager_1 : MonoBehaviour
{
    public MovementAgent<MovementComponent> movement;
    // Start is called before the first frame update
    void Start()
    {

        movement.GetVehicle().Steering().ArriveOn(AgentManager.Instance.GetRole().GetVehicle());
        movement.GetVehicle().Steering().WallAvoidanceOn();
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
}